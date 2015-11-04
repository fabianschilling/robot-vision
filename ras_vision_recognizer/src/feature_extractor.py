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
from cv_bridge import CvBridge


class FeatureExtractor:

    def __init__(self):

        self.node_name = 'feature_extractor'

        cv2.namedWindow('original', cv2.WINDOW_NORMAL)
        cv2.createTrackbar('blur', 'original', 3, 10, self.cb)

        cv2.namedWindow('morphed', cv2.WINDOW_NORMAL)

        cv2.namedWindow('edges', cv2.WINDOW_NORMAL)
        cv2.createTrackbar('thresh1', 'edges', 0, 255, self.cb)
        cv2.createTrackbar('thresh2', 'edges', 0, 255, self.cb)

        self.bridge = CvBridge()

        rospy.init_node(self.node_name, anonymous=True)

        self.subscriber = rospy.Subscriber('vision/object_image', Image, self.image_callback, queue_size=1)

        #self.publisher = rospy.Publisher('vision/object_rect', Rect, queue_size=1)

    def cb(self, x):
        pass

    def image_callback(self, data):

        # Convert from ROS image to OpenCV image
        image = self.bridge.imgmsg_to_cv2(data)

        blur = cv2.getTrackbarPos('blur', 'original')

        if blur > 0:
            image = cv2.blur(image, (blur, blur))

        cv2.imshow('original', image)

        #res = cv2.adaptiveThreshold(original, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 5, 3)

        otsu_thresh, _ = cv2.threshold(image, 0, 255, type=cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        edges = cv2.Canny(image, otsu_thresh * 0.5, otsu_thresh)

        cv2.imshow('morphed', edges)

        # thresh1 = cv2.getTrackbarPos('thresh1', 'edges')
        # thresh2 = cv2.getTrackbarPos('thresh2', 'edges')
        
        # edges = cv2.Canny(res, thresh1, thresh2)

        # cv2.imshow('edges', edges)

        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5, 5))
        # close = cv2.morphologyEx(resized,cv2.MORPH_CLOSE, kernel)
        # div = np.float32(resized) / (close)
        # res = np.uint8(cv2.normalize(div, div, 0, 255, cv2.NORM_MINMAX))

        key = cv2.waitKey(1)
        
        if key == 10: # Return key pressed
            print('Image captured')
            cv2.imwrite('image.jpg', resized)

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
