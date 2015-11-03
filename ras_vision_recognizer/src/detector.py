#!/usr/bin/env python

# Regular imports
import sys
import numpy as np
import scipy
import cv2
from matplotlib import pyplot as plt

# ROS imports
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Detector:

	def __init__(self):

		self.node_name = 'detector'
	
		rospy.init_node(self.node_name, anonymous=True)

		self.subscriber = rospy.Subscriber('camera/rgb/image_raw', Image, self.color_callback, queue_size=1)
		self.subscriber = rospy.Subscriber('camera/depth/image_raw', Image, self.depth_callback, queue_size=1)

		self.bridge = CvBridge()

		cv2.namedWindow('original', cv2.WINDOW_NORMAL)

		cv2.namedWindow('processed', cv2.WINDOW_NORMAL)
		cv2.createTrackbar('blur', 'processed', 3, 10, self.nothing)

		cv2.namedWindow('thresh', cv2.WINDOW_NORMAL)
		cv2.createTrackbar('hue_min', 'thresh', 0, 179, self.nothing)
		cv2.createTrackbar('hue_max', 'thresh', 179, 179, self.nothing)
		cv2.createTrackbar('sat_min', 'thresh', 150, 255, self.nothing)
		cv2.createTrackbar('sat_max', 'thresh', 255, 255, self.nothing)
		cv2.createTrackbar('val_min', 'thresh', 70, 255, self.nothing)
		cv2.createTrackbar('val_max', 'thresh', 255, 255, self.nothing)
		
		cv2.namedWindow('mask', cv2.WINDOW_NORMAL)
		cv2.createTrackbar('dilation', 'mask', 20, 100, self.nothing)
		cv2.createTrackbar('erosion', 'mask', 5, 100, self.nothing)

		# OpenCV variables

		self.scale = 0.8

	def nothing(self, x):
		pass


	def process_original_image(self, image):

		blur = cv2.getTrackbarPos('blur', 'processed')

		if blur > 0:
			# Blur image
			image = cv2.blur(image, (blur, blur))

		return image

	def process_mask(self, mask):

		erosion = cv2.getTrackbarPos('erosion', 'mask')

		if erosion > 0:
			# Erode
			mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (erosion, erosion)))

		dilation = cv2.getTrackbarPos('dilation', 'mask')

		if dilation > 0:
			# Dilate
			mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilation, dilation)))

		return mask

	def color_callback(self, message):

		# Convert from ROS image to OpenCV image
		original = self.bridge.imgmsg_to_cv2(message, 'bgr8')

		# Scale image down
		original = cv2.resize(original, (0, 0), fx=self.scale, fy=self.scale)

		# Process image
		image = self.process_original_image(original)
		cv2.imshow('processed', image)

		# Do color thresholding
		thresh, mask = self.color_threshold(image)
		cv2.imshow('thresh', thresh)

		# Process mask
		mask = self.process_mask(mask)
		cv2.imshow('mask', mask)
		
		# Get contours
		contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

		#for i, contour in enumerate(contours):
		#	cv2.drawContours(original, contours, i, (255, 0, 0))

		largest_contour = self.get_largest_contour(contours)

		if largest_contour is not None:
			x, y, w, h = cv2.boundingRect(largest_contour)
			object_image_color = thresh[y:y + h, x: x + w]
			object_image = original[y:y + h, x: x + w]
			cv2.rectangle(original, (x, y), (x + w, y + h), (255, 255, 255), 2)
			
		cv2.imshow('original', original)

		cv2.waitKey(10)

	def get_largest_contour(self, contours):

		largest_contour_area = 0
		largest_contour = None
		for contour in contours:
			contour_area = cv2.contourArea(contour)
			if contour_area > largest_contour_area:
				largest_contour_area = contour_area
				largest_contour = contour

		return largest_contour

	def color_threshold(self, image):

		hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

		hue_min = cv2.getTrackbarPos('hue_min', 'thresh')
		hue_max = cv2.getTrackbarPos('hue_max', 'thresh')
		sat_min = cv2.getTrackbarPos('sat_min', 'thresh')
		sat_max = cv2.getTrackbarPos('sat_max', 'thresh')
		val_min = cv2.getTrackbarPos('val_min', 'thresh')
		val_max = cv2.getTrackbarPos('val_max', 'thresh')


		lower = np.array([hue_min, sat_min, val_min])
		upper = np.array([hue_max, sat_max, val_max])

		mask = cv2.inRange(hsv_image, lower, upper)

		thresh = cv2.bitwise_and(image, image, mask=mask)

		return (thresh, mask)

	def preprocess(self, image):
		
		# Resizing
		image = cv2.resize(image, (0,0), fx=0.5, fy=0.5)

		# Color to grayscale
		image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		return image

	def depth_callback(self, data):

		# Convert from ROS image to OpenCV image
		image = self.bridge.imgmsg_to_cv2(data)

		# Convert to numpy array
		image = np.array(image, dtype=np.float32)

		# Scale image down
		image = cv2.resize(image, (0,0), fx=self.scale, fy=self.scale)

		# Normalize depth image to range [0, 1]
		cv2.normalize(image, image, 0, 1, cv2.NORM_MINMAX)

		cv2.imshow('depth', image)
		cv2.waitKey(1)
		

def main(args):
	print('Running... Press CTRL-C to quit.')
	detector = Detector()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print('Quitting')
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
