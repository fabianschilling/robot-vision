#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy

from vision_msgs.msg import Histogram

class Visualizer:

	def __init__(self):
		
		rospy.init_node("histogram", anonymous=True)
		self.subscriber = rospy.Subscriber('/vision/histogram', Histogram, self.histogram_callback)
		self.histogram = None
		self.counter = 0


	def histogram_callback(self, histogram):

		print(str(self.counter + 1) + ' histograms received.')

		if self.histogram is None:
			self.histogram = histogram.histogram
		else:
			self.histogram = np.add(self.histogram, histogram.histogram)

		if self.counter == 100:
			plt.plot(self.histogram)
			plt.show()

		self.counter += 1

def main():
	Visualizer()
	rospy.spin()

if __name__ == '__main__':
	main()