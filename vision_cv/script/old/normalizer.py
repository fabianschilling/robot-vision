#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy

from vision_msgs.msg import Histogram
from sklearn.preprocessing import normalize		

def main():

	orange = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_data/color/orange/orange.txt')
	yellow = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_data/color/yellow/yellow.txt')
	lightgreen = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_data/color/lgreen/lgreen.txt')
	green = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_data/color/green/green.txt')
	lightblue = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_data/color/lblue/lblue.txt')
	blue = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_data/color/blue/blue.txt')
	purple = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_data/color/purple/purple.txt')
	red = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_data/color/red/red.txt')

	colordata = [orange, yellow, lightgreen, green, lightblue, blue, purple, red]
	colornames = ['orange', 'yellow', 'lightgreen', 'green', 'lightblue', 'blue', 'purple', 'red']
	colorranges = [(2,23), (24, 46), (59, 84), (85, 121), (188, 208), (213, 229), (266, 305), (340, 359)]
	colorcaps = [np.zeros(360) for _ in range(len(colordata))]
	colornorm = [np.zeros(360) for _ in range(len(colordata))]

	for i, d in enumerate(colordata):
		colorcaps[i][(colorranges[i][0]):(colorranges[i][1])] = d[(colorranges[i][0]):(colorranges[i][1])]
		colornorm[i] = normalize(colorcaps[i][:,np.newaxis], axis=0, norm='l1').ravel()
		np.savetxt(colornames[i] + '.txt', colornorm[i])
		print(np.sum(colornorm[i]))
		plt.plot(colorcaps[i], color=colornames[i])

	plt.show()

	#plt.plot(orangecap, color='orange')
	#plt.show()

	# for i, d in enumerate(colordata):
	# 	plt.plot(d, color=colornames[i])
	# plt.show()


if __name__ == '__main__':
	main()