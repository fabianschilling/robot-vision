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

class Recognizer:

	def __init__(self):

		self.node_name = 'recognizer'
	
		rospy.init_node(self.node_name, anonymous=True)

		self.subscriber = rospy.Subscriber('camera/rgb/image_raw', Image, self.color_callback, queue_size=1)
		#self.subscriber = rospy.Subscriber('camera/depth/image_raw', Image, self.depth_callback, queue_size=1)

		self.bridge = CvBridge()

		cv2.namedWindow('color')
		cv2.createTrackbar('Threshold', 'color', 100, 255, self.nothing)

		cv2.namedWindow('edges')
		
		#cv2.namedWindow('depth')

		cv2.namedWindow('hsv')
		cv2.createTrackbar('Hue Min', 'hsv', 0, 179, self.nothing)
		cv2.createTrackbar('Hue Max', 'hsv', 179, 179, self.nothing)
		cv2.createTrackbar('Sat Min', 'hsv', 100, 255, self.nothing)
		cv2.createTrackbar('Sat Max', 'hsv', 255, 255, self.nothing)
		cv2.createTrackbar('Val Min', 'hsv', 150, 255, self.nothing)
		cv2.createTrackbar('Val Max', 'hsv', 255, 255, self.nothing)

		# OpenCV variables

		self.scale = 1.0

	def nothing(self, x):
		pass

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


	def color_callback(self, data):

		# Convert from ROS image to OpenCV image
		original = self.bridge.imgmsg_to_cv2(data, 'bgr8')

		# Scale image down
		original = cv2.resize(original, (0, 0), fx=self.scale, fy=self.scale)

		# Blur image
		#blurred_image = cv2.blur(image, (5, 5))
		image = cv2.GaussianBlur(original, (5, 5), 1)

		# Do color tracking
		res, mask = self.color_tracking(image)

		# Erode & dilate
		image = cv2.erode(image, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))

		image = cv2.dilate(image, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))

		# Get image keypoints and draw them
		#keypoints = self.detect_blobs(mask)
		#image = cv2.drawKeypoints(image, keypoints, np.array([]), (0, 255, 0))

		#largest_keypoint = None
		#largest_keypoint_size = 0.0
		#for keypoint in keypoints:
		#	if keypoint.size > largest_keypoint_size:
		#		largest_keypoint = keypoint
		#		largest_keypoint_size = keypoint.size
		
		#if largest_keypoint is not None:
		#	cv2.drawKeypoints(image, [largest_keypoint], np.array([]), (0, 255, 0))
		
		# Get contours
		contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

		largest_contour = self.get_largest_contour(contours)

		bb = 20 # bounding box in pixels

		if largest_contour is not None:
			x, y, w, h = cv2.boundingRect(largest_contour)
			rectangle = original[y - bb:y + h + bb, x - bb: x + w + bb]
			self.classify_object(rectangle)
			cv2.rectangle(original, (x - bb, y - bb), (x + w + bb, y + h + bb), (0, 255, 0), 2)

		#image = self.preprocess(image)

		#image = self.detect_blob(image)

		cv2.imshow('color', original)
		cv2.waitKey(30)

	def classify_object(self, image):

		#cv2.imshow('object', image)

		image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		image = cv2.GaussianBlur(image, (5, 5), 1)

		threshold = cv2.getTrackbarPos('Threshold', 'color')

		edges = cv2.Canny(image, threshold, threshold * 3)

		contours, hierarchy = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

		print('# contours: ' + str(len(contours)))

		for contour in contours:
			#approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
			cv2.drawContours(edges, [contour], 0, (0, 255, 0), -1)

		cv2.imshow('edges', edges)


	def get_largest_contour(self, contours):

		largest_contour_area = 0
		largest_contour = None
		for contour in contours:
			contour_area = cv2.contourArea(contour)
			if contour_area > largest_contour_area:
				largest_contour_area = contour_area
				largest_contour = contour

		return largest_contour

	def color_tracking(self, image):

		hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

		hue_min = cv2.getTrackbarPos('Hue Min', 'hsv')
		hue_max = cv2.getTrackbarPos('Hue Max', 'hsv')
		sat_min = cv2.getTrackbarPos('Sat Min', 'hsv')
		sat_max = cv2.getTrackbarPos('Sat Max', 'hsv')
		val_min = cv2.getTrackbarPos('Val Min', 'hsv')
		val_max = cv2.getTrackbarPos('Val Max', 'hsv')


		lower = np.array([hue_min, sat_min, val_min])
		upper = np.array([hue_max, sat_max, val_max])

		mask = cv2.inRange(hsv_image, lower, upper)

		res = cv2.bitwise_and(image, image, mask=mask)

		cv2.imshow('hsv', res)
		#cv2.imshow('mask', mask)

		return (res, mask)

	def color_histogram(self, image):

		colors = ('b', 'g', 'r')
		for i, col in enumerate(colors):
			histr = cv2.calcHist([image], [i], None, [256], [0, 256])
			plt.plot(histr, color = col)
			plt.xlim([0, 256])
		plt.draw()

	def preprocess(self, image):
		
		# Resizing
		image = cv2.resize(image, (0,0), fx=0.5, fy=0.5)

		# Color to grayscale
		image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		return image

	def detect_blobs(self, image):

		# Set up blob detection parameters
		params = cv2.SimpleBlobDetector_Params()
		params.filterByInertia = False
		params.filterByConvexity = False
		params.filterByColor = True
		params.blobColor = 255 # white
		params.filterByArea = True
		params.minArea = 20.0

		detector = cv2.SimpleBlobDetector(params)
		keypoints = detector.detect(image)
		
		return keypoints

	def detect_edges(self, image):

		pass
		

def main(args):
	print('Running... Press CTRL-C to quit.')
	recognizer = Recognizer()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print('Quitting')
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)