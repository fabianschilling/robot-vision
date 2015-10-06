#!/usr/bin/env python

# Regular imports
import sys
import numpy as np
import scipy
import cv2

# ROS imports
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError

class Follower:

	def __init__(self):

		self.node_name = 'follower'

		rospy.init_node(self.node_name, anonymous=True)
		# self.publisher = rospy.Publisher('image_publisher', Int32)

		# Subscribe to RGB and depth image
		# self.subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback, queue_size=1)
		self.subscriber = rospy.Subscriber('/camera/depth/image', Image, self.depth_callback, queue_size=1)

		# Create the CvBridge object
		self.bridge = CvBridge()

	def image_callback(self, data):
		try:
			image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
		except CvBridgeError, e:
			print(e)

		# grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		#blur_image = cv2.GaussianBlur(grayscale_image, (5,5), 0)
		#edge_image = cv2.Canny(blur_image, 100, 200)

		cv2.imshow('RGB Image', image)
		cv2.waitKey(3)

	def depth_callback(self, data):
		try:
			image = self.bridge.imgmsg_to_cv2(data)
		except CvBridgeError, e:
			print(e)

		# Scale factor by which the image is scaled down
		scale = 1.0 / 4.0

		# Scaling by the scale factor
		image = cv2.resize(image, (0,0), fx=scale, fy=scale)

		# Get scaled width and height
		height, width = image.shape

		# Define an interval between the closest depth value and 
		mindepth = np.nanmin(image)
		maxdepth = mindepth + 0.2 # 5 cm depth
		
		argmindepth = np.nanargmin(image)

		roi_min_x = width
		roi_max_x = 0

		registered = []
		for y in range(height):
			for x in range(width):
				value = image[y, x]
				if np.isnan(value):
					continue
				if value >= mindepth and value <= maxdepth:
					registered.append(x)
					if x < roi_min_x:
						roi_min_x = x
					elif x > roi_max_x:
						roi_max_x = x

		if roi_min_x < width and roi_max_x > 0:
			#cv2.rectangle(image, (roi_min_x, 0), (roi_max_x, height), (0, 0, 255))
			#position = (roi_max_x + roi_min_x) / 2
			position = int(np.mean(registered))
			cv2.line(image, (position, 0), (position, height), (0, 255, 0), 3)
			middle = width / 2
			delta = None
			if position >= middle:
				delta = position - middle
			elif position < middle:
				delta = -(middle - position)

			print('Closest object ' + str(int(mindepth * 100.0)) + 'cm away.')
			print('Delta: ' + str(delta))


		# Draw circle around closest object
		#cv2.circle(image, (closest_object_index[1], closest_object_index[0]), 40, (0,0,255))

		#print('Closest object ' + str(mindepth) + 'm away')

		# Used to normalize the image to values between 0..1
		# image_normalized = image / np.nanmax(image)

		# blur_image = cv2.GaussianBlur(image, (13,13), 0)
		# blurred_image = cv2.medianBlur(image, 5)

		cv2.imshow('Depth Image', image)
		cv2.waitKey(3)

def main(args):
	print('Running... Press CTRL-C to quit.')
	follower = Follower()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print('Shutting down.')
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)