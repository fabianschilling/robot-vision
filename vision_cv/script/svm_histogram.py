#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
import sys

from vision_msgs.msg import Histogram
from sklearn.preprocessing import normalize


class Visualizer:

	def __init__(self):
		
		rospy.init_node("histogram", anonymous=True)
		self.subscriber = rospy.Subscriber('/vision/histogram', Histogram, self.histogram_callback)
		self.n_samples = 1000
		self.n_features = 360
		self.data = np.zeros((self.n_samples, self.n_features)) # n_samples * n_features
		self.counter = 0


	def histogram_callback(self, message):

		print(str(self.counter + 1) + ' histograms received.')

		self.data[self.counter, :] = message.histogram

		self.counter += 1

		if self.counter >= self.n_samples:
			np.savetxt('raw_data.txt', self.data)
			print('Data saved.')
			mean_data = np.mean(self.data, axis=0)
			plt.plot(mean_data)
			plt.show()
			self.subscriber.unregister()
		

def main():
	Visualizer()
	rospy.spin()

if __name__ == '__main__':
	main()