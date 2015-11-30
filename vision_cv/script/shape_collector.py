#!/usr/bin/env python

import numpy as np
import cv2
import rospy

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection
from cv_bridge import CvBridge

from sklearn import preprocessing

class ShapeCollector:

	def __init__(self):

		rospy.init_node('shape_collector', anonymous=True)

		self.color_image = None
		self.count = 0
		self.n_samples = 1000
		self.n_features = 30 * 30
		self.size = 30
		self.data = np.zeros((self.n_samples, self.n_features))

		self.color_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.color_callback, queue_size=1)
		self.detection_subscriber = rospy.Subscriber('/vision/detection', Detection, self.detection_callback, queue_size=1)

		self.bridge = CvBridge()

		cv2.namedWindow('image', cv2.WINDOW_NORMAL)

	def cb(self, d):
		pass
	
	def color_callback(self, message):

		self.color_image = self.bridge.imgmsg_to_cv2(message, 'bgr8')

	def detection_callback(self, message):

		if self.color_image is None:
			return

		box = message.box
		centroid = message.centroid

		cropped = self.color_image[box.y:box.y + box.size, box.x:box.x + box.size]

		gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)

		blurred = cv2.GaussianBlur(gray, (5, 5), 1)

		thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

		resized = cv2.resize(thresh, (self.size, self.size))

		binarized = preprocessing.binarize(resized)

		flattened = binarized.flatten()

		self.data[self.count, :] = flattened

		print(str(self.count + 1) + ' samples saved.')

		self.count += 1

		if self.count == self.n_samples:
			np.savetxt('data.txt', self.data)
			print('Data saved.')
			self.detection_subscriber.unregister()
			self.color_subscriber.unregister()

		cv2.imshow('image', resized)

		cv2.waitKey(1)

		
def main():
	ShapeCollector()
	rospy.spin()


if __name__ == '__main__':
	main()