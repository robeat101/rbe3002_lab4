#!/usr/bin/env python

import rospy, tf
import Movement
from nav_msgs.msg import GridCells, OccupancyGrid
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped
from numpy import ma
from math import sqrt
from __builtin__ import pow
from genpy.rostime import Duration


def heuristic(current, end):
    global h_const
    x_2 = pow((current.point.x - end.point.x), 2)
    y_2 = pow((current.point.y - end.point.y), 2)
    h = sqrt(x_2+y_2)
    return h / h_const

#takes in Start and End AStar nodes.
def AStar_search(start, end):
    global map_width
    global map_height
    global map_cell_width
    global map_cell_height
    global map_resolution
    global map_x_offset
    global map_y_offset
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
            #Return path (less one garbage node that is appended) 
        
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

def ObstacleExpansion(msg, robot_resolution):
    if(robot_resolution >= round(msg.info.resolution, 3)):
        return msg
    else:
        return msg

#Map callback function
def map_function(msg):
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
    msg = ObstacleExpansion(msg, robot_resolution)
    map_data = msg.data
    map_width = msg.info.width
    map_height = msg.info.height
    map_resolution = round(msg.info.resolution, 3)
    map_cell_width = map_resolution
    map_cell_height = map_resolution
    map_x_offset = round(msg.info.origin.position.x + (0.5 * map_resolution), 1)
    map_y_offset = round(msg.info.origin.position.y + (0.5 * map_resolution),1)

    diagonal_distance = sqrt(map_cell_width**2 + map_cell_height**2)


#this needs to be universalized for other maps
def getMapIndex(node):
    global map_width
    global map_x_offset
    global map_y_offset
    global map_resolution
    #tiles were being "seen" as one to the right of where they actually are -> the +0.5
    print "node.point.y: " + str(node.point.y)
    print "map_y_offset: " + str(map_y_offset)
    print "map_resolution: " + str(map_resolution)
    a = (((node.point.y - map_y_offset) / map_resolution) * map_width)
    a = a + ((node.point.x - map_x_offset) / map_resolution)
    return int(round(a,2))

#Takes in current node, returns list of possible directional movements
def WhereToGo(node):
    global map_cell_width
    global map_cell_height
    global diagonal_distance
    possibleNodes = []

    direction = Direction()
    #Hacky code begins
    North = AStarNode(round(node.point.x,3), round(node.point.y+map_cell_height,3))
    North.g = node.g + map_cell_height
    North.step_direction = direction.n
    
    NorthEast = AStarNode(round(node.point.x+map_cell_width,3), round(node.point.y+map_cell_height,3))
    NorthEast.g = node.g + diagonal_distance
    NorthEast.step_direction = direction.ne
    
    East = AStarNode(round(node.point.x+map_cell_width,3), round(node.point.y, 3))
    East.g = node.g + map_cell_width
    East.step_direction = direction.e
    
    SouthEast = AStarNode(round(node.point.x+map_cell_width,3), round(node.point.y-map_cell_height,3))
    SouthEast.g = node.g + diagonal_distance
    SouthEast.step_direction = direction.se
    
    South = AStarNode(round(node.point.x, 3), round(node.point.y-map_cell_height,3))
    South.g = node.g + map_cell_height
    South.step_direction = direction.s
    
    SouthWest = AStarNode(round(node.point.x-map_cell_width,3), round(node.point.y-map_cell_height,3))
    SouthWest.g = node.g + diagonal_distance
    SouthWest.step_direction = direction.sw
    
    West = AStarNode(round(node.point.x-map_cell_width,3), round(node.point.y, 3))
    West.g = node.g + map_cell_width
    West.step_direction = direction.w
    
    NorthWest = AStarNode(round(node.point.x-map_cell_width,3), round(node.point.y+map_cell_height,3))
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
    global map_cell_width
    global map_cell_height
    global diagonal_distance
    #determine if diagonal
    #if change in x
    diagonal = round(abs(node.point.x - next.point.x),3) == round(map_cell_width, 3)
    #if change in x and change in y
    diagonal = diagonal and round(abs(node.point.y - next.point.y),3) == round(map_cell_height,3)
    if diagonal:
        #print diagonal_distance
        return round(2 * map_resolution, 3)
    else:
        return round(map_resolution, 3)
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
        self.s = 2
        self.e = 3
        self.w = 4
        self.ne = 5
        self.se = 6
        self.nw = 7
        self.sw = 8
        self.start = 0
        
def getWaypoints(path):
    waypoints = []
    waypoints.append(start)
    direction = Direction()
    previous = start
    previous.direction = direction.start
    
    for node in path:
        if(node.step_direction != previous.step_direction):
            waypoints.append(node)
            previous = node
        else:
            continue
    if len(waypoints) > 1: 
        a = list()
        a.append(waypoints[0])
        a.extend(waypoints[2::])
    waypoints.append(end)
    
    AStar_Done = True
    return waypoints    

#Publish Explored Cells function
def PublishGridCells(publisher, nodes):
    global map_cell_width
    global map_cell_height
    #Initialize gridcell
    gridcells = GridCells()
    gridcells.header.frame_id = 'map'
    gridcells.cell_width = map_cell_width
    gridcells.cell_height = map_cell_height

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
	gridcells.cell_width = map_cell_width
	gridcells.cell_height = map_cell_height

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
            for j in yrange(0, 160):
                a = getMapIndex(node)
                if(a == 100):
                    obstacle.add(node)
                if(a == -1):
                    unknown.add(node)
                if(a == 0):
                    free.add(node)    
                node.point.y = node.point.y + map_resolution
            node.point.x = node.point.x + map_resolution
                
                
    else:
        print "Displaying Path"
        PublishGridCells(pub_path, path)
        rospy.sleep(rospy.Duration(1, 0))
        
        PublishGridCells(pub_path, [])
        
        print "Showing Waypoints"
        waypoints = getWaypoints(path)
        PublishWayPoints(pub_path, waypoints)
        

#####################################3
# set and print initialpose
def set_initial_pose (msg):
    global map_cell_width
    global map_cell_height
    global start_pos_x
    global start_pos_y
    global start_pos_flag
    global end_pos_flag
    #global start_pos_z
    #global start_orient_x
    #global start_orient_y
    #global start_orient_z
    #global start_w
    
    start_pos_x = msg.pose.pose.position.x
    start_pos_y = msg.pose.pose.position.y
    #set initial pose values
    if msg.pose.pose.position.x % map_cell_width < (map_cell_width / 2.0):
    	start_pos_x = start_pos_x - (msg.pose.pose.position.x % (map_cell_width)) #+ (map_cell_width / 2.0)
    else:
    	start_pos_x = start_pos_x - (msg.pose.pose.position.x % map_cell_width) + map_cell_width #+ (3.0 * map_cell_width / 2.0)
    
    if msg.pose.pose.position.y % map_cell_height < (map_cell_height / 2.0):
    	start_pos_y = start_pos_y - (msg.pose.pose.position.y % (map_cell_width)) #+ (map_cell_width / 2.0)
    else:
    	start_pos_y = start_pos_y - (msg.pose.pose.position.y % map_cell_width) + map_cell_width#+ (3.0 * map_cell_width / 2.0)
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
    if(start_pos_flag == True and end_pos_flag == True):
           print 'running Astar'
           run_Astar();
           start_pos_flag = False
           end_pos_flag = False

# set and print goalpose
def set_goal_pose (msg):
    global map_cell_width
    global map_cell_height
    global start_pos_flag
    global end_pos_flag

    end_pos_x = msg.pose.position.x
    end_pos_y = msg.pose.position.y
    #set initial pose values
    if msg.pose.position.x % map_cell_width < (map_cell_width / 2.0):
        end_pos_x = end_pos_x - (msg.pose.position.x % map_cell_width)
    else:
        end_pos_x = end_pos_x - (msg.pose.position.x % map_cell_width) + map_cell_width
    
    if msg.pose.position.y % map_cell_height < (map_cell_height / 2.0):
        end_pos_y = end_pos_y - (msg.pose.position.y % map_cell_height)
    else:
        end_pos_y = end_pos_y - (msg.pose.position.y % map_cell_height) + map_cell_height
        
        
    end.point.x = round(end_pos_x,3)
    end.point.y = round(end_pos_y,3)
    
    print "end_pos_x = ", end.point.x
    print "end_pos_y = ", end.point.y 
    
    end_pos_flag = True
    if(start_pos_flag == True and end_pos_flag == True):
           print 'running Astar'
           run_Astar();
           start_pos_flag = False
           end_pos_flag = False
    
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
    
    global diagonal_distance
    global h_const
    global map_data
    global start
    global end
    global waypoints
    
    global start_pos_flag
    global end_pos_flag
    global AStar_Done
    global robot_resolution
    robot_resolution = 0.2
    AStar_Done = False
    
    
   
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
    pub_map      = rospy.Publisher('/map_optimized', OccupancyGrid)
    
    #Subscribers:
    sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, set_initial_pose, queue_size=1)
    sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, set_goal_pose, queue_size=1)  
    sub = rospy.Subscriber('/map', OccupancyGrid, map_function, queue_size=1)
    
    pub_map.publish(new_map)
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
    
    
    


    

        
    
