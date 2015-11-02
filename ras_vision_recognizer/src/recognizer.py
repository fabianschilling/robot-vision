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

		cv2.namedWindow('original', cv2.WINDOW_NORMAL)

		cv2.namedWindow('processed', cv2.WINDOW_NORMAL)
		#cv2.createTrackbar('dilation', 'processed', 0, 10, self.nothing)
		#cv2.createTrackbar('erosion', 'processed', 0, 10, self.nothing)
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

		cv2.namedWindow('object', cv2.WINDOW_NORMAL)
		cv2.createTrackbar('blur', 'object', 3, 10, self.nothing)

		cv2.namedWindow('edges', cv2.WINDOW_NORMAL)
		cv2.createTrackbar('threshold1', 'edges', 100, 1000, self.nothing)
		cv2.createTrackbar('threshold2', 'edges', 200, 1000, self.nothing)

		# OpenCV variables

		self.scale = 0.3

		plt.ion()
		plt.show()

	def nothing(self, x):
		pass


	def process_original_image(self, image):

		blur = cv2.getTrackbarPos('blur', 'processed')

		if blur > 0:
			# Blur image
			image = cv2.blur(image, (blur, blur))
			#image = cv2.medianBlur(image, (blur, blur))
			#image = cv2.GaussianBlur(image, (blur, blur), 1)

		#erosion = cv2.getTrackbarPos('erosion', 'processed')

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

	def histogram(self, image):

		#hist = cv2.calcHist([image], [0], None, [256], [0, 256])

		plt.hist(image.ravel(), 256, [0, 256])
		#plt.show()
		plt.draw()
		plt.pause(0.01)

	def color_callback(self, message):

		# Convert from ROS image to OpenCV image
		original = self.bridge.imgmsg_to_cv2(message, 'bgr8')

		# Scale image down
		original = cv2.resize(original, (0, 0), fx=self.scale, fy=self.scale)

		# Process image
		image = self.process_original_image(original)
		cv2.imshow('processed', image)

		#self.histogram(image)

		# Do color thresholding
		thresh, mask = self.color_threshold(image)
		cv2.imshow('thresh', thresh)

		# Process mask
		mask = self.process_mask(mask)
		cv2.imshow('mask', mask)

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

		#for i, contour in enumerate(contours):
		#	cv2.drawContours(original, contours, i, (255, 0, 0))

		largest_contour = self.get_largest_contour(contours)

		if largest_contour is not None:
			x, y, w, h = cv2.boundingRect(largest_contour)
			object_image = thresh[y:y + h, x: x + w]
			#self.classify_object(rectangle)
			color_name, color_bgr, prob = self.classify_color(object_image)
			cv2.rectangle(original, (x, y), (x + w, y + h), color_bgr, 2)
			#cv2.putText(original, color + '(' + str(prob) + ')', (x + w, y + h), cv2.FONT_HERSHEY_TRIPLEX, 0.2, 0)

		cv2.imshow('original', original)

		cv2.waitKey(10)

	def classify_color(self, object_image):

		cv2.imshow('object', object_image)

		hsv_image = cv2.cvtColor(object_image, cv2.COLOR_BGR2HSV)

		hist = cv2.calcHist([hsv_image], [0], None, [180], [1, 179])

		total = float(np.sum(hist))
		orange = np.sum(hist[0:12])
		yellow = np.sum(hist[12:31])
		green = np.sum(hist[31:70])
		blue = np.sum(hist[70:120])
		purple = np.sum(hist[120:162])
		red = np.sum(hist[162:179])

		colors = [orange, yellow, green, blue, purple, red]
		color_names = ['orange', 'yellow', 'green', 'blue', 'purple', 'red']
		color_hues = [6, 22, 50, 95, 140, 170]
		color_rgb = [(255, 255/2, 0), (255, 255, 0), (0, 255, 0), (0, 0, 255), (255, 0, 255), (255, 0, 0)]
		color_bgr = [(0, 255/2, 255), (0, 255, 255), (0, 255, 0), (255, 0, 0), (255, 0, 255), (0, 0, 255)]

		color = np.argmax(colors)

		probability = float(colors[color]) / total

		#print('Color: ' + str(color) + '(' + str(probability * 100) + ')')

		return (color_names[color], color_bgr[color], probability)

		#color = hist.argmax()

		#print(color)

		# hue, _, _ = cv2.split(hsv_image)

		# hue_array = hue.flatten()

		# hue_hist, _ = np.histogram(hue_array, bins=32, range=(1, 179))

		# hue_argmax = hue_hist.argmax()

		# print(hue_argmax)

		#cv2.imshow('hue', hue)
		#cv2.imshow('sat', sat)
		#cv2.imshow('val', val)

		#print(hue.shape)

		#color = np.bincount(hue.flatten()).argmax()

		#print('Hue: ' + str(color))
		#print(str(len(hue)))
		#argmaxsat = np.bincount(sat).argmax()
		#argmaxval = np.bincount(val).argmax()

		# if color >= 0 and color < 3:
		# 	print('red')
		# elif color >= 3 and color < 12:
		# 	print('orange')
		# elif color >= 12 and color < 31:
		# 	print('yellow')
		# elif color >= 31 and color < 70:
		# 	print('green')
		# elif color >= 70 and color < 120:
		# 	print('blue')
		# elif color >= 120 and color < 162:
		# 	print('purple')
		# elif color >= 162 and color < 179:
		# 	print('red')

		#print('(h: ' + str(argmaxhue) + ', s: ' + str(argmaxsat) + ', v: ' + str(argmaxval))

		#print('argmax(hue) = ' + str(argmaxhue))

	def classify_object(self, image):

		image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		#image = cv2.GaussianBlur(image, (5, 5), 1)

		blur = cv2.getTrackbarPos('blur', 'object')

		if blur > 0:
			image = cv2.blur(image, (blur, blur))

		cv2.imshow('object', image)

		#otsu_thresh, _ = cv2.threshold(image, 0, 255, type=cv2.THRESH_BINARY + cv2.THRESH_OTSU)

		threshold1 = cv2.getTrackbarPos('threshold1', 'edges')
		threshold2 = cv2.getTrackbarPos('threshold2', 'edges')

		#edges = cv2.Canny(image, otsu_thresh / 2, otsu_thresh)
		edges = cv2.Canny(image, threshold1, threshold2)

		#circles = cv2.HoughCircles(image, cv2.cv.CV_HOUGH_GRADIENT, 2, 100, otsu_thresh)

		#if circles is not None:

			#circles = np.round(circles[0, :].astype('int'))

			#for (x, y, r) in circles:
				#cv2.circle(image, (x, y), r, (0, 255, 0), 4)
				#cv2.putText(image, 'circle', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.2, 0)

		#contours, hierarchy = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

		#print('# contours: ' + str(len(contours)))

		#for contour in contours:
			#approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
		#	cv2.drawContours(edges, [contour], 0, (0, 255, 0), -1)

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
	recognizer = Recognizer()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print('Quitting')
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
