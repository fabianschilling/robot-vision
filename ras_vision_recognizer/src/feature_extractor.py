#!/usr/bin/env python

# Regular imports
import sys
import numpy as np
import cv2

# Scikit learn

from sklearn import preprocessing

# ROS imports
import rospy
from sensor_msgs.msg import Image
from ras_vision_recognizer.msg import Rect
from cv_bridge import CvBridge


class FeatureExtractor:

    def __init__(self):

        self.node_name = 'feature_extractor'

        cv2.namedWindow('original', cv2.WINDOW_NORMAL)
        cv2.createTrackbar('blur', 'original', 0, 20, self.cb)

        cv2.namedWindow('canny', cv2.WINDOW_NORMAL)
        cv2.createTrackbar('lower', 'canny', 0, 255, self.cb)
        cv2.createTrackbar('upper', 'canny', 0, 255, self.cb)

        self.bridge = CvBridge()

        rospy.init_node(self.node_name, anonymous=True)

        self.image = None

        self.count = 0

        self.subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.color_callback, queue_size=1)
        self.subscriber = rospy.Subscriber('vision/object_rect', Rect, self.object_callback, queue_size=1)

        #self.publisher = rospy.Publisher('vision/object_rect', Rect, queue_size=1)

    def cb(self, x):
        pass

    def color_callback(self, data):

        self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def object_callback(self, d):

        # Return if no image available
        if self.image is None:
            return

        # Crop the object
        image = self.image[d.y:d.y + d.height, d.x:d.x + d.width]

        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        image = cv2.blur(image, (3, 3))

        cv2.imshow('original', image)

        mask = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 3, 2)

        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, (5, 5))

        resized = cv2.resize(opening, (30, 30))

        cv2.imshow('canny', resized)

        key = cv2.waitKey(1)
        
        if key == 10: # Return key pressed
            filename = 'images/image' + str(self.count + 1) + '.jpg'
            cv2.imwrite(filename, resized)
            print('Image saved: ' + filename)
            self.count += 1

def main():
    print('Running... Press CTRL-C to quit.')
    FeatureExtractor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Quitting')
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
