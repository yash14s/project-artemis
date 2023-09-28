#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int8
import time
import cv2
import numpy as np

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

def detectDark(image):
	lower_black = 0
	upper_black = 195
	black_mask = cv2.inRange(image, lower_black, upper_black)
	return black_mask

def calcAvgBrightness(p,image):
	mask = np.zeros(image.shape[:2], dtype=np.uint8)
	cv2.rectangle(mask, (p[1], p[2]), (p[3],p[4]),(p[5],p[6]),255, -1)
	return cv2.mean(image, mask)[0]

def scoreCalculator(p,image):
	mask = np.zeros(image.shape[:2], dtype=np.uint8)
	cv2.rectangle(mask, (p[1], p[2]), (p[3],p[4]),(p[5],p[6]),255, -1)
	brightness = cv2.mean(image, mask)[0]
	distance_rating = p[7]
	k_b = 1
	k_d = 5
	score = k_b*brightness + k_d*distance_rating
	return score

def findSector(image):
	result = image.copy()
	h,w = image.shape[:2]
	#Format(number,x,y,a,b,c,d,distance_rating,score) where (x,y) is the center, (a,b) is top-left, (c,d) is bottom-right
	#Distance_rating - E=5,D=F=4,B=H=3,G=I=2,A=C=1
	points = [
		[1,w//6,h//6,0,0,w//3,h//3,1,0],
		[2,w//2,h//6,w//3,0,2*w//3,h//3,3,0],
		[3,5*w//6,h//6,2*w//3,0,w,h//3,1,0],
		[4,w//6,h//2,0,h//3,w//3,2*h//3,4,0],
		[5,w//2,h//2,w//3,h//3,2*w//3,2*h//3,5,0],
		[6,5*w//6,h//2,2*w//3,h//3,w,2*h//3,4,0],
		[7,w//6,5*h//6,0,2*h//3,w//3,h,2,0],
		[8,w//2,5*h//6,w//3,2*h//3,2*w//3,h,3,0],
		[9,5*w//6,5*h//6,2*w//3,2*h//3,w,h,2,0]
	]
	for point in points:
		point[8] = scoreCalculator(point,image)#calcAvgBrightness([point[0],point[1],point[2],point[3],point[4],point[5]],image)
	sorted_points = sorted(points, key=lambda tup: tup[8],reverse=True)
	# returns in format (x,y)
	cv2.rectangle(result, (sorted_points[0][3], sorted_points[0][4]), (sorted_points[0][5], sorted_points[0][6]), (0, 0, 0), 2)
	cv2.imshow('Best sector', result)
	best_sector_center = [sorted_points[0][1],sorted_points[0][2]]
	best_sector = sorted_points[0][0]
	return best_sector

def process_frame(left_matcher, right_matcher, wls_filter):
	while True:
		global left_frame, right_frame

		disparity_left = np.int16(left_matcher.compute(left_frame, right_frame))
		disparity_right = np.int16(right_matcher.compute(right_frame, left_frame))

		disparity_image = wls_filter.filter(disparity_left, left_frame, None, disparity_right)
		disparity_image = cv2.normalize(src=disparity_image, dst=disparity_image, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
		disparity_image = np.uint8(disparity_image)
		cv2.imshow("Disparity", disparity_image)

		thresholded_image = detectDark(disparity_image)
		best_sector = findSector(thresholded_image)
		print(best_sector)

		best_sector_msg = Int8()
		best_sector_msg.data = best_sector
		best_sector_publisher.publish(best_sector_msg)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

rospy.init_node('best_sector_detection_node', anonymous=False)
rospy.Subscriber('/multisense_sl/camera/left/image_raw', Image, left_callback)
rospy.Subscriber('/multisense_sl/camera/right/image_raw', Image, right_callback)
best_sector_publisher = rospy.Publisher('/best_sector_topic', Int8, queue_size = 10)

left_matcher, right_matcher, wls_filter = setup_stereo()
time.sleep(3)

process_frame(left_matcher, right_matcher, wls_filter)
cv2.waitKey(0)
cv2.destroyAllWindows()