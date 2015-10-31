#!/usr/bin/env python

# Regular imports
import sys
import numpy as np
import scipy
import cv2

# ROS imports
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Recognizer:

	def __init__(self):

		self.node_name = 'recognizer'
	
		rospy.init_node(self.node_name, anonymous=True)

		self.subscriber = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback, queue_size=1)

		self.bridge = CvBridge()

	def callback(self, data):

		try:
			image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
		except CvBridgeError, e:
			print(e)

		image = self.preprocess(image)

		image = self.detect_blob(image)



		cv2.imshow('RGB Image', image)
		cv2.waitKey(1)

	def preprocess(self, image):
		
		# Resizing
		image = cv2.resize(image, (0,0), fx=0.5, fy=0.5)

		# Color to grayscale
		image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		return image

	def detect_blob(self, image):

		# Set up blob detection parameters
		params = cv2.SimpleBlobDetector_Params()
		params.filterByArea = True
		params.minArea = 50

		detector = cv2.SimpleBlobDetector(params)
		keypoints = detector.detect(image)
		
		return cv2.drawKeypoints(image, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

	def detect_edges(self, image):

		pass


def main(args):
	print('Running... Press CTRL-C to quit.')
	recognizer = Recognizer()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print('Quitting.')
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)