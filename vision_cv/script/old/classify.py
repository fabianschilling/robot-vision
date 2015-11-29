#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
import sys

from vision_msgs.msg import Histogram
from sklearn.preprocessing import normalize

class Visualizer:

	def __init__(self):

		orange = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_data/color/old/orange/orange.txt')
		yellow = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_data/color/old/yellow/yellow.txt')
		lightgreen = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_data/color/old/lgreen/lgreen.txt')
		green = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_data/color/old/green/green.txt')
		lightblue = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_data/color/old/lblue/lblue.txt')
		blue = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_data/color/old/blue/blue.txt')
		purple = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_data/color/old/purple/purple.txt')
		red = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_data/color/old/red/red.txt')
		
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