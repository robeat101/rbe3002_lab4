#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
import tf
import numpy
import math 

Movement_Stop_flag = False

#This function consumes linear and angular velocities
#and creates a Twist message.  This message is then published.
def publishTwist(lin_vel, ang_vel):
    global pub

    twist_msg = Twist();		#Create Twist Message

    twist_msg.linear.x = lin_vel	#Populate message with data
    twist_msg.angular.z = ang_vel

    #print "Publishing " + str(twist_msg)

    pub.publish(twist_msg)		#Send Message
 
#This function accepts two wheel velocities and a time interval.
#Note: Clarification was given that u1 and u2 should be phi1 phi2,
#however permission was given to use this implementation should it already be written.
def spinWheels(u1, u2, time):
    global pub

    lin_vel = 0.5 * (u1 + u2)			#Determines the linear velocity of base based on the wheels
    ang_vel = (1 / .352) * (u1 - u2)		#Determines the angular velocity of base bread on the wheels.

    twist_msg = Twist();			#Creates two messages: a name-maker and a program killer
    stop_msg = Twist();

    twist_msg.linear.x = lin_vel		#Populate messages with data.
    twist_msg.angular.z = ang_vel
    stop_msg.linear.x = 0
    stop_msg.angular.z = 0
    
    #While the specified amount of time has not elapsed, send Twist messages.
    now = rospy.Time.now().secs
    while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
        pub.publish(twist_msg)
    pub.publish(stop_msg)
    

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global odom_list
    global pose 	

    x0 = pose.pose.position.x	#Set origin
    y0 = pose.pose.position.y

    #Loop until the distance between the attached frame and the origin is equal to the
    #distance specifyed 
    done = False

    while (not done and not rospy.is_shutdown() and Movement_Stop_flag == False):
        x1 = pose.pose.position.x
        y1 = pose.pose.position.y
        d = math.sqrt( (x1 - x0)**2 + (y1 - y0)**2 )	#Distance formula
        
        print "  " + str(distance) + " " + str(d)

        if (d >= distance):
            done = True
            publishTwist(0, 0)
        else:
            publishTwist(speed, 0)

        rospy.sleep(rospy.Duration(0, 500000))

    #Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    global odom_list
    global pose
    
    print angle
    start_angle = robotyaw
    twist = Twist()
    twist.linear.x = 0; 
    twist.linear.y = 0; twist.angular.y = 0 
    twist.linear.z = 0; twist.angular.x = 0
    if angle > 0:
        twist.angular.z = 1;
    else:
        twist.angular.z = -1;
        
    print "basic calcs"
    current_angle = robotyaw
    while(not(abs(current_angle - start_angle) < abs(angle) + 0.2 and abs(current_angle - start_angle) > abs(angle) - 0.2)):
        pub.publish(twist)
        current_angle = robotyaw
        rospy.sleep(rospy.Duration(.2, 0))
    print "Done while loop"
    twist.angular.z = 0;
    pub.publish(twist)
    return


#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    global odom_list

    w = speed / radius
    v1 = w * (radius + .5*.352)
    v2 = w * (radius - .5*.352)

    transformer = tf.TransformerROS()	
    rotation = numpy.array([[math.cos(angle), -math.sin(angle), 0],
                            [math.sin(angle), math.cos(angle), 0],
                            [0,          0,          1]])
    (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    T_o_t = transformer.fromTranslationRotation(trans, rot)
    R_o_t = T_o_t[0:3,0:3]
    goal_rot = numpy.dot(rotation, R_o_t)

    done = False
    while (not done and not rospy.is_shutdown()):
        #Gets transforms
        (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        state = transformer.fromTranslationRotation(trans, rot)
        state_rot = state[0:3,0:3]

        #Decides if they are within tolerance or not
        within_tolerance = abs((state_rot - goal_rot)) < .1
        if (within_tolerance.all()):
            spinWheels(0,0,0)
            done = True
            print "done"
        else:
            if (angle > 0):
                spinWheels(v1,v2,.1)
            else:
                spinWheels(v2,v1,.1)

#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    stepSleep = rospy.Duration(.5)
    driveStraight(.25, .6)
    rospy.sleep(stepSleep)
    rotate(-90)
    rospy.sleep(stepSleep)
    driveArc(.25, .1, 180)
    rospy.sleep(stepSleep)
    rotate(135)
    rospy.sleep(stepSleep)
    driveStraight(.25, .42)

#Odometry Callback function.
def readOdom(msg):
    global pose
    global odom_tf
    global robotyaw
    pose = msg.pose
    geo_quat = pose.pose.orientation
  
    odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0),
       (pose.pose.orientation.x, pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w),
       rospy.Time.now(),
       "base_footprint",
       "odom")
    
    robotorientation = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    eulerOrientation = tf.transformations.euler_from_quaternion(robotorientation)
    robotyaw = eulerOrientation[2] + 3.14
    
    #odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0), (pose.pose.orientation.x, pose.pose.orientation.y, 
#pose.pose.orientation.z, pose.pose.orientation.w), rospy.Time.now(), "base_footprint", "odom")


def returnTransform_mapOdom():

    global odom_list

    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
    return odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))


#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        print "Bumper Pressed"
        rospy.Timer(rospy.Duration(.01), timerCallback)
        executeTrajectory()

#The timer is used when it is saving data to csv files for analysis.
def timerCallback(event):
    global pose

    x = pose.pose.position.x
    y = pose.pose.position.y
    quaternion = (pose.pose.orientation.x,
                  pose.pose.orientation.y,
                  pose.pose.orientation.z,
                  pose.pose.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)
    theta = euler[2]


#Main handler of the project
def movement_init():
    global pub
    global pose
    global odom_tf
    global odom_list

    cmds = [[1, 0], [2, 0], [0.5, 0], [0, 1], [0, 2], [0, 0], [1, 3.14]]
    odom_list = tf.TransformListener()
    odom_tf = tf.TransformBroadcaster()
    
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)
    sub = rospy.Subscriber('/odom', Odometry, readOdom, queue_size=5)
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper)


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
    #rospy.sleep(sleeper)

    #driveStraight(.1, 1)
    #print "Drive straight"
    #rotate(math.pi/2)
    #print "Drive rotated"
    #driveArc(.5, .5, math.pi / 2)
    #print "Drive Arc'd"
    #executeTrajectory()
    #spinWheels(0.125, .25, 2)
    #rospy.sleep(sleeper)
    #rospy.loginfo("Complete")
    


if __name__ == '__main__':
    try:
        movement_init()
    except rospy.ROSInterruptException:
        pass
