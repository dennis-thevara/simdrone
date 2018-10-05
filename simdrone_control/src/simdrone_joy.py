#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sys import exit

class SimdroneJoynode(object):

    def __init__(self):
        # Setup Publishers:
        self.takeoff_pub = rospy.Publisher("ardrone/takeoff", Empty, queue_size=1)
        self.land_pub = rospy.Publisher("ardrone/land", Empty, queue_size=1)
        self.reset_pub = rospy.Publisher("ardrone/reset", Empty, queue_size=1)
        self.joypub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # Create twist and empty data variables:
        self.twist = Twist()
        self.empty = Empty()
        
        # Initialize ROS Node:
        rospy.init_node("simdrone_joynode", anonymous=True)

        # Subscribe to joystick messages:
        self.sub = rospy.Subscriber("joy", Joy, self.joypub_callback, queue_size=1)

    def joypub_callback(self,joypose):
        # Create class variable:
        self.data = joypose
        # Takeoff:
        if self.data.buttons[2] == 1:
            self.takeoff()
        # Land:
        elif self.data.buttons[3] == 1:
            self.land()
        # Reset:
        elif self.data.buttons[7] == 1:
            self.reset()
        # Linear in XY:
        elif (self.data.axes[4] != 0 or self.data.axes[3] != 0):
            self.linearxy()
        # Yaw:
        elif self.data.axes[0] != 0:
            self.yaw()
        # Climb:
        elif self.data.buttons[5] == 1:
            self.elev(1)
        # Drop:
        elif self.data.buttons[4] == 1:
            self.elev(-1)
        # Neutral:
        else:
            self.neutral()

    def takeoff(self):
        # Send takeoff command:
        rospy.loginfo("Taking Off!")
        self.takeoff_pub.publish(self.empty)

    def land(self):
        # Send land command:
        rospy.loginfo("Landing!")
        self.land_pub.publish(self.empty)

    def reset(self):
        # Send reset command:
        rospy.loginfo("Resetting!")
        self.reset_pub.publish(self.empty)

    def linearxy(self):
        # Linear translation in X:
        self.twist.linear.x = self.data.axes[4]
        self.twist.linear.y = self.data.axes[3]
        self.joypub.publish(self.twist)

    def yaw(self):
        # Angular rotation in Z
        self.twist.angular.z = self.data.axes[0]
        self.joypub.publish(self.twist)

    def elev(self,val):
        # Linear translation in Z
        self.twist.linear.z = val
        self.joypub.publish(self.twist)

    def neutral(self):
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        self.joypub.publish(self.twist)

if __name__ == "__main__":
    try:
        SimdroneJoynode()
        rospy.spin()
    except rospy.ROSInterruptException:
        exit(0)