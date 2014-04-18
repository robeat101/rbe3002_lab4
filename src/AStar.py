#!/usr/bin/env python

import rospy, tf
import Movement
import copy
from nav_msgs.msg import GridCells, OccupancyGrid
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, Twist, PointStamped
from numpy import ma
from math import sqrt
from __builtin__ import pow
from genpy.rostime import Duration


def heuristic(current, end):
    global h_const
    x_2 = pow((current.point.x - end.point.x), 2)
    y_2 = pow((current.point.y - end.point.y), 2)
    h = sqrt(x_2+y_2)
    return 20 * h / h_const

#takes in Start and End AStar nodes.
def AStar_search(start, end):
    global map_scaled_width
    global map_scaled_height
    global map_scaled_cell_width
    global map_scaled_cell_height
    global map_scaled_resolution
    global map_scaled_x_offset
    global map_scaled_y_offset
    global pub_frontier
    #Set Start and End values
    start.g = 0
    start.h = heuristic(start, end)
    rospy.sleep(rospy.Duration(0.1,0))
    
    #FrontierSet is the set of nodes to be opened, i.e. Frontier
    FrontierSet = set()
    #ExpandedSet is the set of nodes already expanded
    ExpandedSet = set()
    
    #Add Start to frontier
    FrontierSet.add(start)
    
    while FrontierSet:
        PublishGridCells(pub_frontier, FrontierSet)
        PublishGridCells(pub_explored, ExpandedSet)
        PublishGridCells(pub_start, [start])
        PublishGridCells(pub_end, [end])
        #find the node in FrontierSet with the minimum heuristic value
        current = min(FrontierSet, key=lambda o:o.g + o.h)
        #If the goal is being expanded
        if current.poseEqual(end):
            #Construct path
            path = []
            repeatedNode_flag = False
            FrontierSet.remove(current)
            while current.parent:
                path.append(current)
                current = current.parent
                #remove current node from ExpandedSet
                for expanded in ExpandedSet:
                    if current.poseEqual(expanded):
                        ExpandedSet.remove(expanded)
                        break
                path.append(current)
            for frontier in FrontierSet:
                if end.poseEqual(frontier):
                    FrontierSet.remove(frontier)

            rospy.sleep(rospy.Duration(.2,0))
            #update gridcells
            PublishGridCells(pub_explored, ExpandedSet)
            PublishGridCells(pub_frontier, FrontierSet)
            #Return path (less one garbage node that is appended and less the start node)
            return path[1:-1]
        #Else, move node from frontier to explored
        FrontierSet.remove(current)
        if not (current.poseEqual(start) or current.poseEqual(end)):
            ExpandedSet.add(current)
        
        #Check for possible 8-directional moves
        repeatedNode_flag = False
        somethingWasUpdatedFlag = False
        
        nodesToGo = WhereToGo(current)
        if nodesToGo == []:
            return []
            
        for node in nodesToGo:
            #Ignore if node is expanded
            for expanded in ExpandedSet:
                if node.poseEqual(expanded):
                    new_g = current.g + move_cost(current,node)
                    if node.g > new_g:
                        node.g = new_g
                        node.h = heuristic(node, end)
                        node.parent = current
                    repeatedNode_flag = True
                    somethingWasUpdatedFlag = True
                    break   
            #Try to update cost of traveling to node if already exists
            for frontier in FrontierSet:
                if node.poseEqual(frontier) and repeatedNode_flag == False:
                    new_g = current.g + move_cost(current,node)
                    if node.g > new_g:
                        node.g = new_g
                        node.h = heuristic(node, end)
                        node.parent = current
                        FrontierSet.remove(frontier)
                        FrontierSet.add(node)
                    somethingWasUpdatedFlag = True
                    break
                if somethingWasUpdatedFlag == True:
                    break
            #Add to frontier and update costs and heuristic values
            if somethingWasUpdatedFlag == False:
                node.g = current.g + move_cost(current, node)
                node.h = heuristic(node, end)
                node.parent = current
                FrontierSet.add(node)
            repeatedNode_flag = False
            somethingWasUpdatedFlag = False
    return None

def ObstacleExpansion(map):
    global pub_map
    global robot_resolution
    
    if(robot_resolution < round(map.info.resolution, 3)):
        return map
    else:
        target_exp = int(round(robot_resolution/map.info.resolution, 0))
        new_map = copy.copy(map)
        
        MapPosStatus = {}
        for x in xrange(0, map.info.width):
            for y in xrange(0, map.info.height):
                index = y * map.info.width + x
                MapPosStatus[(x,y)] = map.data[index]
        xy = []
        for i in xrange(-target_exp, target_exp+1):
            for j in xrange(-target_exp, target_exp+1):
                xy.append((i,j));
        new_data = list(new_map.data)
        
        
        for x in xrange(0, map.info.width):
            for y in xrange(0, map.info.height):
                nodeI = y * map.info.width + x
                if(MapPosStatus[x,y] == 100):
                    for node in xy:
                         index = (y + node[1]) * map.info.width + (x + node[0])
                         if(index >= 0 and index < len(new_data)):
                                new_data[index] =  100
        new_map.data = tuple(new_data)
        pub_map.publish(new_map)
        rospy.sleep(rospy.Duration(4,0))
        return new_map
    
                

#Map callback function
def map_function(msg):
    
   # print "Starting Map_function"
    global map_data
    global map_width
    global map_height
    global map_cell_width
    global map_cell_height
    global map_resolution
    global map_x_offset
    global map_y_offset
    global diagonal_distance
    global robot_resolution
    global curr_odom_x
    global curr_odom_y

    #for Occupancy Grid Optimization
    global map_scale
    global map_scaled_width
    global map_scaled_height
    global map_scaled_cell_width
    global map_scaled_cell_height
    global map_scaled_resolution #set to robot_resolution
    global map_scaled_x_offset
    global map_scaled_y_offset
    
    msg = ObstacleExpansion(msg)
    map_data = msg.data
    map_resolution = round(msg.info.resolution, 3)
    map_width = msg.info.width
    map_height = msg.info.height
    map_cell_width = map_resolution
    map_cell_height = map_resolution
    map_x_offset = round(msg.info.origin.position.x + (0.5 * map_resolution), 1)
    map_y_offset = round(msg.info.origin.position.y + (0.5 * map_resolution), 1)


    """curr_odom_x = msg.pose.pose.readOdom().x
    curr_odom_y = msg.pose.pose.readOdom().y
    start_pos_x = curr_odom_x.pub_start
    start_pos_y = curr_odom_y.pub_start

    print start_pos_y
    print start_pos_x   """
    
    





    #for Occupancy Grid Optimization
    map_scale = int(round(map_scaled_resolution / map_resolution)) #scale factor
    new_map = copy.copy(msg)
    
    new_map.info.width = int(round(map_width / map_scale, 0))
    new_map.info.height = int(round(map_height / map_scale, 0))
    new_map.info.resolution = map_scaled_resolution 
    
    map_scaled_width = new_map.info.width
    map_scaled_height = new_map.info.height
    map_scaled_cell_width = map_scaled_resolution
    map_scaled_cell_height = map_scaled_resolution
    
    map_scaled_x_offset = map_x_offset
    map_scaled_y_offset = map_y_offset
    diagonal_distance = sqrt(map_scaled_cell_width**2 + map_scaled_cell_height**2)
    
    MapPosStatus = {}
    for x in xrange(0, map_scaled_width):
        for y in xrange(0, map_scaled_height):
            index = y * map_scaled_width + x
            MapPosStatus[(x,y)] = 0
    xy = []
    for i in xrange(0, map_scale):
        for j in xrange(0, map_scale):
            xy.append((i,j));
    
    new_data = [0,] * (map_scaled_width * map_scaled_height)
    for x in xrange(0, map_scaled_width):
        for y in xrange(0, map_scaled_height):
            cost = 0
            nodes = 0
            for node in xy:
                nodex = x * map_scale + node[0]
                nodey = y * map_scale + node[1]
                index = nodey * map_width + nodex
                if(index >= 0 and index < len(map_data)):
                    if(map_data[index] >= 0):
                        cost = cost + map_data[index]
                    else:
                        cost = cost + 20
                    nodes = nodes+1
            cost = cost/nodes
            if(cost > 75):
                cost = 100
            new_data[y * map_scaled_width + x] = cost
            
    
    new_map.data = tuple(new_data)
    map_data = new_map.data
    pub_map.publish(new_map)
    print "Done"
    if(end_pos_flag == True):
        w = run_Astar
        if(w != None):
         
        	print run_Astar()
       		
        	#movement_init()
            #DriveTowards(w[0])
        

#this needs to be universalized for other maps
def getMapIndex(node):
    global map_scaled_width
    global map_scaled_x_offset
    global map_scaled_y_offset
    global map_scaled_resolution
    #tiles were being "seen" as one to the right of where they actually are -> the +0.5
    #print "node.point.y: " + str(node.point.y)
    #print "map_y_offset: " + str(map_y_offset)
    #print "map_resolution: " + str(map_resolution)
    a = (((node.point.y - map_scaled_y_offset) / map_scaled_resolution) * map_scaled_width)
    a = a + ((node.point.x - map_scaled_x_offset) / map_scaled_resolution)
    return int(round(a,2))

#Takes in current node, returns list of possible directional movements
def WhereToGo(node):
    global map_scaled_cell_width
    global map_scaled_cell_height
    global diagonal_distance
    possibleNodes = []

    direction = Direction()
    #Hacky code begins
    North = AStarNode(round(node.point.x,3), round(node.point.y+map_scaled_cell_height,3))
    North.g = node.g + map_scaled_cell_height
    North.step_direction = direction.n
    
    NorthEast = AStarNode(round(node.point.x+map_scaled_cell_width,3), round(node.point.y+map_scaled_cell_height,3))
    NorthEast.g = node.g + diagonal_distance
    NorthEast.step_direction = direction.ne
    
    East = AStarNode(round(node.point.x+map_scaled_cell_width,3), round(node.point.y, 3))
    East.g = node.g + map_scaled_cell_width
    East.step_direction = direction.e
    
    SouthEast = AStarNode(round(node.point.x+map_scaled_cell_width,3), round(node.point.y-map_scaled_cell_height,3))
    SouthEast.g = node.g + diagonal_distance
    SouthEast.step_direction = direction.se
    
    South = AStarNode(round(node.point.x, 3), round(node.point.y-map_scaled_cell_height,3))
    South.g = node.g + map_scaled_cell_height
    South.step_direction = direction.s
    
    SouthWest = AStarNode(round(node.point.x-map_scaled_cell_width,3), round(node.point.y-map_scaled_cell_height,3))
    SouthWest.g = node.g + diagonal_distance
    SouthWest.step_direction = direction.sw
    
    West = AStarNode(round(node.point.x-map_scaled_cell_width,3), round(node.point.y, 3))
    West.g = node.g + map_scaled_cell_width
    West.step_direction = direction.w
    
    NorthWest = AStarNode(round(node.point.x-map_scaled_cell_width,3), round(node.point.y+map_scaled_cell_height,3))
    NorthWest.g = node.g + diagonal_distance
    NorthWest.step_direction = direction.nw

    if (map_data[getMapIndex(North)] != 100) and (map_data[getMapIndex(North)] != -1):
        possibleNodes.append(North)
    if (map_data[getMapIndex(NorthEast)] != 100) and (map_data[getMapIndex(NorthEast)] != -1):
        possibleNodes.append(NorthEast)
    if (map_data[getMapIndex(East)] != 100) and (map_data[getMapIndex(East)] != -1):
        possibleNodes.append(East)
    if (map_data[getMapIndex(SouthEast)] != 100) and (map_data[getMapIndex(SouthEast)] != -1):
        possibleNodes.append(SouthEast)
    if (map_data[getMapIndex(South)] != 100) and (map_data[getMapIndex(South)] != -1):
        possibleNodes.append(South)
    if (map_data[getMapIndex(SouthWest)] != 100) and (map_data[getMapIndex(SouthWest)] != -1):
        possibleNodes.append(SouthWest)
    if (map_data[getMapIndex(West)] != 100) and (map_data[getMapIndex(West)] != -1):
        possibleNodes.append(West)
    if (map_data[getMapIndex(NorthWest)] != 100) and (map_data[getMapIndex(NorthWest)] != -1):
        possibleNodes.append(NorthWest)

    return possibleNodes

def move_cost(node, next):
    global map_scaled_cell_width
    global map_scaled_cell_height
    global diagonal_distance
    
    if(node.step_direction == next.step_direction):
        return round(map_scaled_resolution * (20 + map_data[getMapIndex(next)]), 3)
    else:
        step_diff = abs(node.step_direction - next.step_direction)
        if step_diff >= 4: 
            step_diff = 8 - step_diff
        return round(map_scaled_resolution * (20 + map_data[getMapIndex(next)]), 3) + step_diff * 2
    
class AStarNode():   
    def __init__(self, x, y):
        direction = Direction()
        self.point = Point()
        self.point.x, self.point.y = x, y
        self.g = 0
        self.h = 0
        self.parent = None
        self.step_direction = direction.start
        
    def poseEqual(self, node):
        return_val = round(node.point.x, 3) == round(self.point.x,3)
        return_val = return_val and round(node.point.y, 3) == round(self.point.y,3) 
    
        return return_val

class Direction():
    def __init__(self):
        self.n = 1
        self.ne = 2
        self.e = 3
        self.se = 4
        self.s = 5
        self.sw = 6
        self.w = 7
        self.nw = 8
        self.start = 0
        
def getWaypoints(path):
    waypoints = []
    direction = Direction()
    previous = start
    previous.direction = direction.start
    
    for node in path:
        if(node.step_direction != previous.step_direction):
            waypoints.append(node)
            previous = node
        else:
            continue
    waypoints.append(end)
    
    AStar_Done = True
    return waypoints    

#Publish Explored Cells function
def PublishGridCells(publisher, nodes):
    global map_scaled_cell_width
    global map_scaled_cell_height
    #Initialize gridcell
    gridcells = GridCells()
    gridcells.header.frame_id = 'map'
    gridcells.cell_width = map_scaled_cell_width
    gridcells.cell_height = map_scaled_cell_height
    
    if nodes == None:
        return
    #Iterate through list of nodes
    for node in nodes: 
        point = Point()
        point.x = node.point.x
        point.y = node.point.y
        #Ensure z axis is 0 (2d Map)
        point.z = node.point.z = 0
        gridcells.cells.append(point)        
    publisher.publish(gridcells)
    #rospy.sleep(rospy.Duration(0.05,0))

def PublishWayPoints(publisher, nodes):
	rospy.sleep(rospy.Duration(0.1,0))
	#Initialize gridcell
	gridcells = GridCells()
	gridcells.header.frame_id = 'map'
	gridcells.cell_width = map_scaled_cell_width
	gridcells.cell_height = map_scaled_cell_height

	#Iterate through list of nodes
	for node in nodes: 
		point = Point()
		point.x = node.point.x
		point.y = node.point.y
		#Ensure z axis is 0 (2d Map)
		point.z = node.point.z = 0
		gridcells.cells.append(point)        
	publisher.publish(gridcells)
	rospy.sleep(rospy.Duration(0.1,0))


def run_Astar():
    global pub_start
    global pub_end
    global pub_path
    set_initial_pose()
    PublishGridCells(pub_start, [start])
    PublishGridCells(pub_end, [end])
    PublishGridCells(pub_path, [])
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(.1, 0))
    
    path = AStar_search(start, end)
    print "h, change_x, change_y"
    print start.h
    print end.point.x - start.point.x
    print end.point.y - start.point.y
    
    if path == []:
        print "No Path!"
        node = AStarNode(-20, -20)
        free = set()
        unknown = set()
        obstacle = set()
        for i in xrange(0, 190):
            for j in xrange(0, 160):
                a = getMapIndex(node)
                if(a == 100):
                    obstacle.add(node)
                if(a == -1):
                    unknown.add(node)
                if(a == 0):
                    free.add(node)    
                node.point.y = node.point.y + map_resolution
            node.point.x = node.point.x + map_resolution
        return None
                
                
    else:
        print "Displaying Path"
        PublishGridCells(pub_path, path)
        rospy.sleep(rospy.Duration(1, 0))
        
        PublishGridCells(pub_path, [])
        
        print "Showing Waypoints"
        waypoints = getWaypoints(path)
        PublishWayPoints(pub_path, waypoints)
        return waypoints

#####################################3
# set and print initialpose
def set_initial_pose ():
    global map_scaled_cell_width
    global map_scaled_cell_height
    global start_pos_x
    global start_pos_y
    global start_pos_flag
    global end_pos_flag
    global robot_resolution
    
    (trans, rot) = Movement.returnTransform_mapOdom()
    print trans
    print rot
    
    #rospy.spin()
    start.point.x = trans[0]
    start.point.y = trans[1]
    start.orientation = rot

    pose_x = trans[0]
    pose_y = trans[1]
 
    
    map_scaled_cell_width = robot_resolution
    map_scaled_cell_height = robot_resolution
    
    #global start_pos_z
    #global start_orient_x
    #global start_orient_y
    #global start_orient_z
    #global start_w
    
    start_pos_x = pose_x
    start_pos_y = pose_y
    #set initial pose values
    if pose_x % map_scaled_cell_width < (map_scaled_cell_width / 2.0):
    	start_pos_x = start_pos_x - (pose_x % (map_scaled_cell_width)) #+ (map_cell_width / 2.0)
    else:
    	start_pos_x = start_pos_x - (pose_x % map_scaled_cell_width) + map_scaled_cell_width #+ (3.0 * map_cell_width / 2.0)
    
    if pose_y % map_scaled_cell_height < (map_scaled_cell_height / 2.0):
    	start_pos_y = start_pos_y - (pose_y % (map_scaled_cell_width)) #+ (map_cell_width / 2.0)
    else:
    	start_pos_y = start_pos_y - (pose_y % map_scaled_cell_width) + map_scaled_cell_width#+ (3.0 * map_cell_width / 2.0)
    #if msg.pose.pose.position.x % .2 < .1:
    #	start_pos_x = (msg.pose.pose.position.x / .2)
    #else:
    #	start_pos_x = (msg.pose.pose.position.x / .2) + .2
    
    #we have not checked scaling or anything for these:
    #start_orient_x = msg.pose.pose.orientation.x
    #start_orient_y = msg.pose.pose.orientation.y
    #start_orient_z = msg.pose.pose.orientation.z
    #start_w = msg.pose.pose.orientation.w
    
    #print initial pose values
    start.point.x = round(start_pos_x, 3)
    start.point.y = round(start_pos_y, 3)
    print ""
    print "Initial Pose Values:"
    print "start_pos_x = ", start.point.x
    print "start_pos_y = ", start.point.y
    #print "start_pos_z = ", start_pos_z
    
   
    
    start_pos_flag = True
    #we have not checked scaling or anything for these:
    #print "start_orient_x = ", start_orient_x
    #print "start_orient_y = ", start_orient_y
    #print "start_orient_z = ", start_orient_z
    #print "start_w = ", start_w


# set and print goalpose
def set_goal_pose (msg):
    global map_scaled_cell_width
    global map_scaled_cell_height
    global start_pos_flag
    global end_pos_flag
    global robot_resolution
    
    map_scaled_cell_width = robot_resolution
    map_scaled_cell_height = robot_resolution
    end_pos_x = msg.point.x
    end_pos_y = msg.point.y
    #set initial pose values
    if msg.point.x % map_scaled_cell_width < (map_scaled_cell_width / 2.0):
        end_pos_x = end_pos_x - (msg.point.x % map_scaled_cell_width)
    else:
        end_pos_x = end_pos_x - (msg.point.x % map_scaled_cell_width) + map_scaled_cell_width
    
    if msg.point.y % map_scaled_cell_height < (map_scaled_cell_height / 2.0):
        end_pos_y = end_pos_y - (msg.point.y % map_scaled_cell_height)
    else:
        end_pos_y = end_pos_y - (msg.point.y % map_scaled_cell_height) + map_scaled_cell_height
        
        
    end.point.x = round(end_pos_x,3)
    end.point.y = round(end_pos_y,3)
    
    print "end_pos_x = ", end.point.x
    print "end_pos_y = ", end.point.y 
    set_initial_pose()
     
    
    
    start_pos_flag = True
    end_pos_flag = True
    if(start_pos_flag == True and end_pos_flag == True):
           print 'running Astar'
           try:
               waypoints = run_Astar();
           except:
               pass
  

#-------------------------//-----------------------------------------------------------//----------------
# Robot Drive Functions

def pubTwist(linear, angular):
  global pub_tf
  global odom_tf

  twist_msg = Twist();		#Create Twist Message

  twist_msg.linear.x = linear	#Populate message with data
  twist_msg.angular.z = angular


  pub_tf.publish(twist_msg)	



def motorDrive_waypoint(): 
	global pub
	global pose
	global odom_tf
	global odom_list

	cmds = [[1, 0], [2, 0], [0.5, 0], [0, 1], [0, 2], [0, 0], [1, 3.14]]

	pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)
	sub = rospy.Subscriber('odom', Odometry, readOdom, queue_size=5)
	bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper)
	odom_list = tf.TransformListener()
	odom_tf = tf.TransformBroadcaster()

	if not odom_tf:
		print "odom_tf not initalized properly. Exiting."
		return
	else: 
		print odom_tf

	odom_tf.sendTransform((0, 0, 0),
	(0, 0, 0, 1),
	rospy.Time.now(),
	"base_footprint",
	"odom")
	sleeper = rospy.Duration(1)
	rospy.sleep(sleeper)

	#driveStraight(.1, 1)
	#print "Drive straight"
	#rotate(math.pi/2)
	#print "Drive rotated"
	#driveArc(.5, .5, math.pi / 2)
	#print "Drive Arc'd"
	#executeTrajectory()
	#spinWheels(0.125, .25, 2)
	rospy.sleep(sleeper)
	rospy.loginfo("Complete")



def astar_init():
        # Change this node name to include your username
    rospy.init_node('rbansal_vcunha_dbourque_Lab3Node')
    
    
    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables
    
    global publisher
    global pub_start
    global pub_end
    global pub_path
    global pub_frontier
    global pub_explored
    global pose
    global odom_tf
    global odom_list

    global map_width
    global map_height
    global map_cell_width
    global map_cell_height
    global map_resolution
    global map_x_offset
    global map_y_offset

    #for Occupancy Grid Optimization
    global map_scale
    global map_scaled_width
    global map_scaled_height
    global map_scaled_cell_width
    global map_scaled_cell_height
    global map_scaled_resolution
    global map_scaled_x_offset
    global map_scaled_y_offset
    global map_scaled_flag
    
    global diagonal_distance
    global h_const
    global map_data
    global start
    global end
    global waypoints
    global pub_map
    global start_pos_flag
    global end_pos_flag
    global AStar_Done
    global robot_resolution

    map_scaled_flag = False
    robot_resolution = 0.2
    map_scaled_resolution = robot_resolution
    AStar_Done = True
    
    
   
    #global start_pos_z
    #global start_orient_x
    #global start_orient_y
    #global start_orient_z
    #globalstart_w
    global goal_pos_x
    global goal_pos_y
    #global goal_pos_z
    #global goal_orient_x
    #global goal_orient_y
    #global goal_orient_z
    #global goal_w
    start_pos_flag = False
    end_pos_flag = False
    
    waypoints = []

    #f(n) = g(n) + h'(n)
    #h'(n) = h(n) / h_const
    #    h_const < 1 -> f(n) becomes heuristic dominant = greedy
    #    h_const > 1 -> f(n) becomes movement cost dominant = optimal search (more time!!)
    #        2 -> seems safe enough
    h_const = 1
        
    #Publishers: 
    pub_start    = rospy.Publisher('/start', GridCells) # Publisher for start Point
    pub_end      = rospy.Publisher('/end'  , GridCells) # Publisher for End Point
    pub_path     = rospy.Publisher('/path' , GridCells) # Publisher for Final Path
    pub_explored = rospy.Publisher('/explored', GridCells) # Publisher explored GridCells
    pub_frontier = rospy.Publisher('/frontier', GridCells) # Publisher explored GridCells
    pub_map      = rospy.Publisher('/map_optimized', OccupancyGrid, latch=True)
    
    #Subscribers:
    #sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, set_initial_pose, queue_size=1)
    sub = rospy.Subscriber('/clicked_point', PointStamped, set_goal_pose, queue_size=1)  
    sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, map_function, queue_size=1)
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    
    print "Starting Lab 4"
    end = AStarNode(1,1.8);
    start = AStarNode(-1,-1.8)
    print "AStar set up. "


#######################################
# This is the program's main function
if __name__ == '__main__':
    astar_init()
    Movement.movement_init()
    
    rospy.spin()
    
    
    


    

        
    
