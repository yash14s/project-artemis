#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tensorflow as tf
import time

bridge = CvBridge()

def rgb_callback(frame):
	global bridge
	global rgb_frame
	try:
		rgb_frame = bridge.imgmsg_to_cv2(frame, "bgr8")
	except CvBridgeError as e:
		print(e)

def detectPoacher(image):
	classes = None
	with open("/home/yash/catkin_ws/src/project_artemis/src/yolo/coco.names", "r") as f:
		classes = [line.strip() for line in f.readlines()]

	net = cv2.dnn.readNet('/home/yash/catkin_ws/src/project_artemis/src/yolo/yolov3.weights', '/home/yash/catkin_ws/src/project_artemis/src/yolo/yolov3.cfg')
	net.setInput(cv2.dnn.blobFromImage(image, 0.00392, (416,416), (0,0,0), True, crop=False))
	layer_names = net.getLayerNames()
	output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
	outs = net.forward(output_layers)

	class_ids = []
	confidences = []
	boxes = []
	Width = image.shape[1]
	Height = image.shape[0]
	for out in outs:
		for detection in out:
			scores = detection[5:]
			class_id = np.argmax(scores)
			confidence = scores[class_id]
			if confidence > 0.1:
				center_x = int(detection[0] * Width)
				center_y = int(detection[1] * Height)
				w = int(detection[2] * Width)
				h = int(detection[3] * Height)
				x = int(center_x - w // 2)
				y = int(center_y - h // 2)
				class_ids.append(class_id)
				confidences.append(float(confidence))
				boxes.append([x, y, w, h])

	indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.6, 0.6)#check if is people detection
	humanCount = 0
	k = 0
	for i in indices:
		i = i[0]
		box = boxes[i]
		if class_ids[i] in [0,15,16,20,21,22,23]:
			color = (0,0,0)
			confidence = round(confidences[i],2)
			if 0.7<=confidence<=1:
				color = (0,255,0) #green
			elif 0.4<=confidence<0.7:
				color = (0,255,255) #yellow
			else:
				color = (0,0,255) #red
			
			if class_ids[i]==0:
				humanCount += 1
				k += confidence
				#if color == (0,255,0):
				 #   k += confidence
				#elif color == (0,255,255):
				 #   k += 0.9*confidence
				#elif color == (0,0,255):
				 #   k += 0.1*confidence
				
			label = str(classes[class_ids[i]]) +": "+str(confidence) #str(classes[class_id])
			cv2.rectangle(image, (int(round(box[0])),int(round(box[1]))), (int(round(box[0]+box[2])),int(round(box[1]+box[3]))), color, 1)
			cv2.putText(image, label, (int(round(box[0])-10),int(round(box[1])-10)), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1)
			#humanCount += 1
	#print('Total Humans Detected:',humanCount)
	#plt.imshow(image)
	if humanCount == 0:
		k ==0
	else:
		k = k/humanCount
	return image,k

def processVideoPoacher():
	# Loop until the end of the video
	font                   = cv2.FONT_HERSHEY_SIMPLEX
	position               = (10,500)
	fontScale              = 1
	fontColor              = (255,255,0)

	suspectLevel = 0
	k1 = 0
	while True:
		# Capture frame-by-frame
		global rgb_frame
		frame = cv2.resize(rgb_frame, (540, 380), fx = 0, fy = 0,interpolation = cv2.INTER_CUBIC)
		# Display the resulting frame
		#cv2.imshow('Frame', frame)
		detectedFrame, k = detectPoacher(frame)
		if k>=0.8 and suspectLevel<5:
			suspectLevel += 1
		elif 0.4<=k<0.8 and suspectLevel < 5:
			suspectLevel += 0.5
		elif 0.2<=k<0.4 and suspectLevel < 5:
			suspectLevel += 0.1
		elif k<0.2 and suspectLevel>0:
			suspectLevel -= 1
		#k1 = k
		text = 'No Activity'
		fontColor = (0,255,0)
		if (2<=suspectLevel<5):
			text = 'Suspicious activity detected!'
			fontColor = (0,255,255)
			print('Suspicious activity detected!',suspectLevel)
			#suspectLevel = 0
			#k1 = 0
		elif (suspectLevel>=5):
			fontColor = (0,0,255)
			text = 'Humans detected!'
			print('Humans detected!',suspectLevel)
			#suspectLevel = 0
			#k1 = 0
			
		cv2.putText(detectedFrame,text,(13,24),font,fontScale,fontColor)
		#print(k,val)
		cv2.imshow('Thresh', detectedFrame)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

rospy.init_node('poacher_detection_node', anonymous=False)
rospy.Subscriber('/rgb_cam/image_raw', Image, rgb_callback)

time.sleep(3)
sess = tf.compat.v1.Session()
sess.run(processVideoPoacher())
cv2.waitKey(0)
cv2.destroyAllWindows()
