#!/usr/bin/env python

import numpy as np
import cv2
import os

import rospkg
import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection
from vision_msgs.msg import Object
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

		self.color_names = {-1: 'no color', 0:'orange', 1:'yellow', 2:'lightgreen',3: 'green', 4:'lightblue', 5:'blue', 6:'purple', 7:'red'}
		self.color_map = {'no color':-1, 'orange': 0, 'yellow':1, 'lightgreen':2, 'green':3, 'lightblue':4, 'blue':5, 'purple':6, 'red':7}
		self.material_names = {-1: 'no material', 0:'wood', 1:'plastic'}
		self.material_map = {'no material': -1, 'wood':0, 'plastic':1}
		self.shape_names = {-1: 'no shape', 0:'ball', 1:'cube'}
		self.shape_map = {'no shape':-1, 'ball':0, 'cube':1}
		self.object_names = {-1: 'no object', 0:'patric', 1:'yellow ball', 2:'yellow cube', 3:'green cylinder', 4:'green cube', 5:'blue triangle', 6:'blue cube', 7:'purple cross', 8:'purple star', 9:'red ball', 10:'red cube', 11:'red hollow cube'}
		self.object_map = {'no object':-1, 'patric':0, 'yellow ball':1, 'yellow cube':2, 'green cylinder':3, 'green cube':4, 'blue triangle':5, 'blue cube':6, 'purple cross':7, 'purple star':8, 'red ball':9, 'red cube':10, 'red hollow cube':11}

		self.color_image = None
		self.depth_image = None
		self.image_evidence = None

		self.detection_subsriber = rospy.Subscriber('/vision/detection', Detection, self.detection_callback, queue_size=1)
		self.color_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.color_callback, queue_size=1)
		self.depth_subscriber = rospy.Subscriber('/camera/depth/image', Image, self.depth_callback, queue_size=1)

		self.object_publisher = rospy.Publisher('/vision/object', Object, queue_size=1)

		self.bridge = CvBridge()


	def color_callback(self, message):
		self.color_image = self.bridge.imgmsg_to_cv2(message, 'bgr8')

	def depth_callback(self, message):
		self.depth_image = np.array(self.bridge.imgmsg_to_cv2(message), dtype=np.float32)

	def send_detection_message(self, centroid, color, material=-1, shape=-1, objecttype=-1, stop=False):

		msg = Object()
		msg.x = centroid.x
		msg.y = centroid.y
		msg.z = centroid.z
		msg.image = self.bridge.cv2_to_imgmsg(self.image_evidence, encoding='passthrough')
		msg.color = color
		msg.shape = shape
		msg.material = material
		msg.objecttype = objecttype
		msg.stop = stop

		self.object_publisher.publish(msg)

	def detection_callback(self, message):

		print('detection callback')

		histogram = message.histogram.histogram
		box = message.box
		centroid = message.centroid

		# If too far away, only classify color
		if centroid.z > 0.65 or centroid.z < 0.4:
			color = self.classify_color(histogram)
			self.send_detection_message(centroid, color)
			#print(self.color_names[color] + ' object. Too far away to classify so get closer.')
		else:
			color = self.classify_color(histogram)
			material = self.classify_material(box, centroid)
			shape = self.classify_shape(box)
			objecttype = self.classify_object(color, material, shape)
			if objecttype != -1:
				print(self.object_names[objecttype])
				self.send_detection_message(centroid, color, material=material, shape=shape, objecttype=objecttype, stop=True)

	def classify_object(self, color, material, shape):

		cm = self.color_map
		mm = self.material_map
		sm = self.shape_map
		om = self.object_map

		if color == cm['lightgreen'] and material == mm['plastic']:
			return om['green cylinder']
		elif color == cm['lightblue'] and material == self.material_map['plastic']:
			return om['blue triangle']
		elif color == cm['yellow'] and material == mm['wood']:
			if shape == sm['cube']:
				return om['yellow cube']
			elif shape == sm['ball']:
				return om['yellow ball']
		elif color == cm['green'] and material == mm['wood']:
			return om['green cube']
		elif color == cm['red'] and material == mm['wood']:
			if shape == sm['cube']:
				return om['red cube']
			elif shape == sm['ball']:
				return om['red ball']
		elif color == cm['blue'] and material == mm['wood']:
			return om['blue cube']
		elif color == cm['purple'] and material == mm['plastic']:
			# TODO: ADD CLASSIFIER FOR STAR/CROSS
			return om['purple cross']
		elif color == cm['orange'] and material == mm['plastic']:
			return om['patric']
		elif color == cm['red'] and material == mm['plastic']:
			return om['red hollow cube']
		else:
			return om['no object']

	def classify_color(self, histogram):
		return int(self.color_clf.predict(histogram))

	def classify_material(self, box, centroid):

		self.image_evidence = self.depth_image[box.y:box.y + box.size, box.x:box.x + box.size]
		
		nans = np.count_nonzero(np.isnan(self.image_evidence))

		nanratio =  nans / float(box.size * box.size)

		X = [nanratio, centroid.z]

		return int(self.material_clf.predict([nanratio, centroid.z]))

	def classify_shape(self, box):

		cropped = self.color_image[box.y:box.y + box.size, box.x:box.x + box.size]

		gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)

		blurred = cv2.GaussianBlur(gray, (5, 5), 1)

		thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

		if thresh is None:
			return
			
		resized = cv2.resize(thresh, (30, 30))

		binarized = preprocessing.binarize(resized)

		flattened = binarized.flatten()

		return int(self.shape_clf.predict(flattened))



def main():
	Classifier()
	rospy.spin()

if __name__ == '__main__':
	main()