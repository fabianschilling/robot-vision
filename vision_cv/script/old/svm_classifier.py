#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy

from vision_msgs.msg import Histogram
from sklearn.preprocessing import normalize
from sklearn import svm
from sklearn.ensemble import RandomForestClassifier
from sklearn.externals import joblib

class Visualizer:

	def __init__(self):
		
		rospy.init_node("histogram", anonymous=True)
		self.subscriber = rospy.Subscriber('/vision/histogram', Histogram, self.histogram_callback)
		#self.clf = joblib.load('/home/fabian/catkin_ws/src/ras_vision/vision_cv/script/svm/redvsgrn.pkl')
		self.clf = joblib.load('/home/fabian/catkin_ws/src/ras_vision/vision_cv/script/svm/redvsgrn.pkl')
		self.counter = 0


	def histogram_callback(self, message):

		X = message.histogram

		print(self.clf.predict_proba(X))

		

def main():
	Visualizer()
	rospy.spin()

if __name__ == '__main__':
	main()