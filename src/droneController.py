#!/usr/bin/env python

'''
Author: Matt Epperson
'''


import sys, thread, time, math
import roslib;

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
#from leap_motion.msg import leapros

class DroneController():
    def __init__(self):

        # Subscribes to the leapros msg
        #rospy.Subscriber("leapmotion/data", leapros, leapCallBack)

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

    def leapCallBack(self, leapros):
        self.command.linear.x = leapros.ypr.y
        self.command.linear.y = leapros.ypr.z
        self.command.linear.z = leapros.ypr.x

def main():

    # Start Node
    rospy.init_node("drone_controller")

    drone = DroneController()

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


