#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

bridge = CvBridge()

def rgb_callback(frame):
	global bridge
	try:
		cv_frame = bridge.imgmsg_to_cv2(frame, "bgr8")
	except CvBridgeError as e:
		print(e)
	
	k = 0.6
	cv_frame = cv2.resize(cv_frame, None, fx= k, fy= k, interpolation= cv2.INTER_LINEAR)
	cv2.imshow("RGB camera",cv_frame)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		cv2.waitKey(0)
		cv2.destroyAllWindows()	
    		
rospy.init_node('camera_subscriber', anonymous=False)
rospy.Subscriber('/rgb_cam/image_raw', Image, rgb_callback)

try:
	rospy.spin()	
except KeyboardInterrupt:
	cv2.destroyAllWindows()
