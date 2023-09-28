#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def callback(lidar_msg):
    altitude = lidar_msg.ranges[0]
    if altitude == float('inf'):
        altitude = 8
    rospy.loginfo("[Lidar] : %f",altitude)

rospy.init_node('lidar_sub', anonymous=True)
rospy.Subscriber("spur/laser/scan", LaserScan, callback)
rospy.spin()

