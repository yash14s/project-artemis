#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import cv2
import numpy as np
from sklearn.preprocessing import normalize

DPI=96
bridge = CvBridge()
kernel_size = 3

def left_callback(frame):
	global bridge
	global left_frame
	try:
		left_frame = bridge.imgmsg_to_cv2(frame, "bgr8")
		left_frame = cv2.GaussianBlur(left_frame, (kernel_size,kernel_size), 1.5)
	except CvBridgeError as e:
		print(e)

def right_callback(frame):
	global bridge, right_frame
	try:
		right_frame = bridge.imgmsg_to_cv2(frame, "bgr8")
		right_frame = cv2.GaussianBlur(right_frame, (kernel_size,kernel_size), 1.5)
	except CvBridgeError as e:
		print(e)

def setup_stereo():
	window_size = 9
	left_matcher = cv2.StereoSGBM_create(
		numDisparities=8,
		blockSize=7,
		P1=8*3*window_size**2,
		P2=32*3*window_size**2,
		disp12MaxDiff=1,
		uniquenessRatio=16,
		speckleRange=2,
		mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
	)
	right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
	wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
	wls_filter.setLambda(80000)
	wls_filter.setSigmaColor(1.2)
	return left_matcher, right_matcher, wls_filter

def process_frame(left_matcher, right_matcher, wls_filter):
	while True:
		global left_frame, right_frame
	
		disparity_left = np.int16(left_matcher.compute(left_frame, right_frame))
		disparity_right = np.int16(right_matcher.compute(right_frame, left_frame) )

		wls_image = wls_filter.filter(disparity_left, left_frame, None, disparity_right)
		wls_image = cv2.normalize(src=wls_image, dst=wls_image, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
		wls_image = np.uint8(wls_image)

		cv2.imshow("Left", left_frame)
		cv2.imshow("Right", right_frame)
		cv2.imshow("Disparity", wls_image)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

rospy.init_node('disparty_image_node', anonymous = False)
rospy.Subscriber('/multisense_sl/camera/left/image_raw', Image, left_callback)
rospy.Subscriber('/multisense_sl/camera/right/image_raw', Image, right_callback)

left_matcher, right_matcher, wls_filter = setup_stereo()
time.sleep(3)

process_frame(left_matcher, right_matcher, wls_filter)
cv2.waitKey(0)
cv2.destroyAllWindows()
