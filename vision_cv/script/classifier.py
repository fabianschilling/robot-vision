#!/usr/bin/env python

import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection
from cv_bridge import CvBridge

from sklearn.ensemble import RandomForestClassifier
from sklearn.externals import joblib
from sklearn import preprocessing

class Classifier:
	
	def __init__(self):
		
		rospy.init_node('classifier', anonymous=True)

		self.classifier_path = '/home/fabian/catkin_ws/src/ras_vision/vision_cv/script/data/'
		self.color_clf = joblib.load(self.classifier_path + 'color_clf/color_clf.pkl')
		self.material_clf = joblib.load(self.classifier_path + 'material_clf/material_clf.pkl')
		self.shape_clf = joblib.load(self.classifier_path + 'shape_clf/shape_clf.pkl')

		self.color_names = ['orange', 'yellow', 'lightgreen', 'green', 'lightblue', 'blue', 'purple', 'red']
		self.material_names = ['wood', 'plastic']
		self.shape_names = ['ball', 'cube']

		self.color_image = None
		self.depth_image = None

		self.color_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.color_callback, queue_size=1)
		self.depth_subscriber = rospy.Subscriber('/camera/depth/image', Image, self.depth_callback, queue_size=1)
		self.detection_subsriber = rospy.Subscriber('/vision/detection', Detection, self.detection_callback, queue_size=1)
		self.detection_publisher = rospy.Publisher('/vision/object')

		self.bridge = CvBridge()


	def color_callback(self, message):
		self.color_image = self.bridge.imgmsg_to_cv2(message, 'bgr8')

	def depth_callback(self, message):
		self.depth_image = np.array(self.bridge.imgmsg_to_cv2(message), dtype=np.float32)

	def detection_callback(self, message):

		histogram = message.histogram.histogram

		color = self.classify_color(message.histogram.histogram)
		material = self.classify_material(message.box, message.centroid)
		shape = self.classify_shape(message.box)

		#print('color: ' + str(color))
		#print('material: ' + str(material))

		if color == 'lightgreen' and material == 'plastic':
			print('green cylinder')
		elif color == 'lightblue' and material == 'plastic':
			print('blue triangle')
		elif color == 'yellow' and material == 'wood':
			if shape == 'cube':
				print('yellow cube')
			elif shape == 'ball':
				print('yellow ball')
		elif color == 'green' and material == 'wood':
			print('green cube')
		elif color == 'red' and material == 'wood':
			if shape == 'cube':
				print('red cube')
			elif shape == 'ball':
				print('red ball')
		elif color == 'blue' and material == 'wood':
			print('blue cube')
		elif color == 'purple' and material == 'plastic':
			print('purple star/cross')
		elif color == 'orange' and material == 'plastic':
			print('patric')
		elif color == 'red' and material == 'plastic':
			print('red hollow cube')
		
		#print(message.histogram)

	def classify_color(self, histogram):
		return self.color_names[int(self.color_clf.predict(histogram))] 
		# return self.color_clf.predict_proba(histogram)

	def classify_material(self, box, centroid):

		cropped = self.depth_image[box.y:box.y + box.size, box.x:box.x + box.size]
		
		nans = np.count_nonzero(np.isnan(cropped))

		nanratio =  nans / float(box.size * box.size)

		X = [nanratio, centroid.z]

		return self.material_names[int(self.material_clf.predict([nanratio, centroid.z]))] 
		# return self.material_clf.predict_proba(X)

	def classify_shape(self, box):

		cropped = self.color_image[box.y:box.y + box.size, box.x:box.x + box.size]

		gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)

		blurred = cv2.GaussianBlur(gray, (5, 5), 1)

		thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

		resized = cv2.resize(thresh, (30, 30))

		binarized = preprocessing.binarize(resized)

		flattened = binarized.flatten()

		#print(self.shape_clf.predict(flattened))

		return self.shape_names[int(self.shape_clf.predict(flattened))]



def main():
	Classifier()
	rospy.spin()

if __name__ == '__main__':
	main()