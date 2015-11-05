#!/usr/bin/env python

# Future imports
from __future__ import print_function

# Regular imports
import sys
import numpy as np
import cv2
import os
import signal

# Scikit learn

from sklearn.preprocessing import scale
from sklearn.externals import joblib
from sklearn.svm import SVC

# ROS imports
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
from ras_vision_recognizer.msg import Rect
from cv_bridge import CvBridge


class ObjectClassifier:

    def __init__(self):

        signal.signal(signal.SIGINT, signal_callback)

        self.node_name = 'shape_recognizer'

        self.bridge = CvBridge()

        rospy.init_node(self.node_name, anonymous=True)

        self.image = None

        self.count = 0

        self.shape = None
        self.color = None
        self.material = None

        self.subscriber = rospy.Subscriber('/object/shape', UInt8, self.shape_callback, queue_size=1)
        self.subscriber = rospy.Subscriber('/object/color', UInt8, self.color_callback, queue_size=1)
        self.subscriber = rospy.Subscriber('/object/material', UInt8, self.material_callback, queue_size=1)

        #self.publisher = rospy.Publisher('vision/object_rect', Rect, queue_size=1)

    def cb(self, x):
        pass

    def shape_callback(self, msg):
        self.shape = msg.data

    def color_callback(self, msg):
        self.color = msg.data

    def material_callback(self, msg):
        self.material = msg.data

        print('Shape: ' + str(self.shape) + ', color: ' + str(self.color) + ', material: ' + str(self.material))

def signal_callback(signal, frame):
    shutdown()

def shutdown():
    sys.stdout.flush()
    print()
    print('Quitting...')
    rospy.signal_shutdown('Quitting...')
    cv2.destroyAllWindows()

def main():
    print('Running... Press CTRL-C to quit.')
    ObjectClassifier()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Quitting')
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
