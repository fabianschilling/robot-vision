#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
import cv2

from vision_msgs.msg import Detection
from sensor_msgs.msg import Image
from sklearn.preprocessing import normalize
from cv_bridge import CvBridge

class MaterialClassifier:

	def __init__(self):
		
		rospy.init_node("histogram", anonymous=True)
		self.image = None
		self.detection_subscriber = rospy.Subscriber('/vision/detection', Detection, self.detection_callback)
		self.depth_subscriber = rospy.Subscriber('/camera/depth/image', Image, self.depth_callback)
		self.bridge = CvBridge()


	def depth_callback(self, message):

		original = self.bridge.imgmsg_to_cv2(message)

		original = np.array(original, dtype=np.float32)

		self.image = original

		# cv2.imshow('depth', original)

		# cv2.waitKey(1)

	def detection_callback(self, message):

		if self.image is None:
			return

		box = message.box

		cropped = self.image[box.y:box.y + box.size, box.x:box.x + box.size]

		cv2.imshow('cropped', cropped)

		nans = np.count_nonzero(np.isnan(cropped))

		nanratio =  nans / float(box.size * box.size)

		print('NaN ratio: ' + str(nanratio))

		cv2.waitKey(1)
		

def main():
	MaterialClassifier()
	rospy.spin()

if __name__ == '__main__':
	main()