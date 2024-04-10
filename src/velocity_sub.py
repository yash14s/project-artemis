#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def callback(velocity_msg):
    rospy.loginfo("vx = %f , vy = %f",velocity_msg.linear.x, velocity_msg.linear.y)

rospy.init_node('velocity_sub', anonymous=True)
rospy.Subscriber("velocity_topic", Twist, callback)
rospy.spin()

