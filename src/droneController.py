#!/usr/bin/env python

'''
Author: Matt Epperson
'''


import sys, thread, time, math
import roslib;

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from leap_motion.msg import leapros

class DroneController():
    def __init__(self):

        # Subscribes to the leapros msg
        rospy.Subscriber("leapmotion/data", leapros, leapCallBack)

        # Empty Message commands
        self.pubLand = rospy.Publisher('ardrone/land', Empty)
        self.pubReset = rospy.Publisher('ardrone/reset', Empty)
        self.pubTakeOff = rospy.Publisher('ardrone/takeoff', Empty)


        # Teleop
        self.pubTwist = rospy.Publisher('ardrone/cmd_vel', Twist)

        # Empty Message
        self.empty = Empty()

        # Hover Message
        self.command_hover = Twist()
        self.command_hover.linear.x = 0;
        self.command_hover.linear.y = 0;
        self.command_hover.linear.z = 0;
        self.command_hover.angular.x = 0;
        self.command_hover.angular.y = 0;
        self.command_hover.angular.z = 0;

        # Command Velocity Message
        self.command = Twist()

        # Land Drone if we shutdown ROS
        rospy.on_shutdown(self.land)

    def takeOff(self):
        self.pubTakeOff.publish(self.empty)

    def reset(self):
        self.pubReset.publish(self.empty)

    def land(self):
        self.pubLand.publish(self.empty)

    def hover(self):
        self.pubTwist.publish(self.command_hover)

def leapCallBack(leapros):
    max_speed = .5
    min_speed = -.5
    dead_band = .1
    lastX = 0
    lastY = 0
    lastZ = 0

    pubCommand = rospy.Publisher('ardrone/cmd_vel', Twist)

    command = Twist()
    command.linear.x = leapros.normal.z
    command.linear.y = leapros.normal.x
    command.linear.z = ((leapros.palmpos.y-200)/450.0)

    ''' SET DEAD BAND '''
    if abs(command.linear.x) < dead_band:
        command.linear.x = 0

    if abs(command.linear.y) < dead_band:
        command.linear.y = 0

    if abs(command.linear.z) < dead_band:
        command.linear.z = 0

    ''' SPEED CONTROL '''
    if command.linear.x > 0:
        command.linear.x = min(command.linear.x, max_speed)
    else:
        command.linear.x = max(command.linear.x, min_speed)

    if command.linear.y > 0:
        command.linear.y = min(command.linear.y, max_speed)
    else:
        command.linear.y = max(command.linear.y, min_speed)

    if command.linear.z > 0:
        command.linear.z = min(command.linear.z, max_speed)
    else:
        command.linear.z = max(command.linear.z, min_speed)

    ''' LAST VALUE CHECK '''
    if leapros.normal.z == lastX:
        command.linear.x = 0
    if leapros.normal.x == lastY:
        command.linear.y = 0
    if leapros.palmpos.z == lastZ:
        command.linear.z = 0

    pubCommand.publish(command)

    lastX = leapros.normal.z
    lastY = leapros.normal.x
    lastZ = leapros.palmpos.z

    print("Commands")
    print( command.linear.x);
    print(command.linear.y);
    print(command.linear.z);
    print ""



def listener():
    rospy.Subscriber("leapros/data", leapros, leapCallBack)


def main():

    # Start Node
    rospy.init_node("drone_controller")

    drone = DroneController()
    listener()

    # Get start time
    start = rospy.get_time()

    # Set Publishing rate
    r = rospy.Rate(30)

    while not rospy.is_shutdown():
        if rospy.get_time() < start+2.0:
            print "Starting Up "
            drone.takeOff()
            drone.hover()

        else:
            drone.land()
        r.sleep()

if __name__ == "__main__":
    main()


