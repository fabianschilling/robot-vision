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
		self.count = 0
		self.n_samples = 10000
		self.n_features = 2
		self.data = np.zeros((self.n_samples, self.n_features))
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
		centroid = message.centroid

		cropped = self.image[box.y:box.y + box.size, box.x:box.x + box.size]

		cv2.imshow('cropped', cropped)

		nans = np.count_nonzero(np.isnan(cropped))

		nanratio =  nans / float(box.size * box.size)

		self.data[self.count, :] = [nanratio, centroid.z]

		print(str(self.count + 1) + ': ratio: ' + str(nanratio) + ', distance: ' + str(centroid.z))

		self.count += 1

		if self.count == self.n_samples:
			np.savetxt('data.txt', self.data)
			print('Data saved.')
			plt.scatter(self.data[:, 0], self.data[:, 1])
			plt.show()
			self.detection_subscriber.unregister()
			self.depth_subscriber.unregister()



		cv2.waitKey(1)
		

def main():
	MaterialClassifier()
	rospy.spin()

if __name__ == '__main__':
	main()