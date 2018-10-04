#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sys import exit

# Setup Publishers:
takeoff_pub = rospy.Publisher("ardrone/takeoff", Empty, queue_size=1)
land_pub = rospy.Publisher("ardrone/land", Empty, queue_size=1)
reset_pub = rospy.Publisher("ardrone/reset", Empty, queue_size=1)
joypub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

# Create twist and empty data variables:
twist = Twist()
empty = Empty()

def simdrone_joynode():
    # Initialize ROS Node:
    rospy.init_node("simdrone_joynode", anonymous=True)

    # Subscribe to joystick messages:
    rospy.Subscriber("joy", Joy, joypub_callback, queue_size=1)

    # Keep node alive until killed:
    rospy.spin()

def joypub_callback(data):

    # Takeoff:
    if data.buttons[2] == 1:
        takeoff()
    # Land:
    elif data.buttons[3] == 1:
        land()
    # Reset:
    elif data.buttons[7] == 1:
        reset()
    # Linear in XY:
    elif (data.axes[4] != 0 or data.axes[3] != 0):
        linearxy(data.axes[4],data.axes[3])
    # Yaw:
    elif data.axes[0] != 0:
        yaw(data.axes[0])
    # Climb:
    elif data.buttons[5] == 1:
        elev(1)
    # Drop:
    elif data.buttons[4] == 1:
        elev(-1)
    # Neutral:
    else:
        neutral()


def takeoff():
    # Send takeoff command:
    rospy.loginfo("Taking Off!")
    takeoff_pub.publish(empty)

def land():
    # Send land command:
    rospy.loginfo("Landing!")
    land_pub.publish(empty)

def reset():
    # Send reset command:
    rospy.loginfo("Resetting!")
    reset_pub.publish(empty)

def linearxy(linx,liny):
    # Linear translation in X:
    twist.linear.x = linx
    twist.linear.y = liny
    joypub.publish(twist)

def yaw(angz):
    # Angular rotation in Z
    twist.angular.z = angz
    joypub.publish(twist)

def elev(val):
    # Linear translation in Z
    twist.linear.z = val
    joypub.publish(twist)

def neutral():
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    joypub.publish(twist)

if __name__ == "__main__":
    try:
        simdrone_joynode()
    except rospy.ROSInterruptException:
        print "Exiting!"
        exit(0)