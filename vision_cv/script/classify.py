#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
import sys

from vision_msgs.msg import Histogram
from sklearn.preprocessing import normalize

class Visualizer:

	def __init__(self):

		orange = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_cv/script/orange.txt')
		yellow = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_cv/script/yellow.txt')
		lightgreen = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_cv/script/lightgreen.txt')
		green = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_cv/script/green.txt')
		lightblue = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_cv/script/lightblue.txt')
		blue = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_cv/script/blue.txt')
		purple = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_cv/script/purple.txt')
		red = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_cv/script/red.txt')
		
		self.colornames = ['orange' , 'yellow' , 'lightgreen' , 'green', 'lightblue', 'blue', 'purple', 'red']
		self.colors = [orange, yellow, lightgreen, green, lightblue, blue, purple, red]

		rospy.init_node("histogram", anonymous=True)
		self.subscriber = rospy.Subscriber('/vision/histogram', Histogram, self.histogram_callback)
		self.histogram = None
		self.counter = 0


	def histogram_callback(self, histogram):

		values = []
		for i in range(len(self.colors)):
			values.append(np.dot(histogram.histogram, self.colors[i]))

		index = np.argmax(values)
		print(self.colornames[index])

def main():
	Visualizer()
	rospy.spin()

if __name__ == '__main__':
	main()