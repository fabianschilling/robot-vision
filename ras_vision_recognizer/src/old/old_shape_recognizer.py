#!/usr/bin/env python

# Regular imports
import sys
import numpy as np
import cv2

# ROS imports
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ShapeRecognizer:

    def __init__(self):

        self.node_name = 'shape_recognizer'

        self.bridge = CvBridge()

        rospy.init_node(self.node_name, anonymous=True)

        self.subscriber = rospy.Subscriber('vision/object_image', Image, self.image_callback, queue_size=1)

        #self.publisher = rospy.Publisher('vision/object_rect', Rect, queue_size=1)

        cv2.namedWindow('original', cv2.WINDOW_NORMAL)

        cv2.namedWindow('resized', cv2.WINDOW_NORMAL)

    def cb(self, x):
        pass

    def image_callback(self, data):

        # Convert from ROS image to OpenCV image
        original = self.bridge.imgmsg_to_cv2(data)

        # Blur image
        blurred = cv2.blur(original, (3, 3))

        otsu_thresh, _ = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        upper = otsu_thresh
        lower = otsu_thresh * 0.2

        edges = cv2.Canny(blurred, lower, upper)

        # Resize image
        #resized = cv2.resize(edges, (50, 50))

        #equalized = cv2.equalizeHist(resized)

        contours, hierarchy = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        largest_contour = self.get_largest_contour(contours)

        cv2.drawContours(original, [largest_contour], -1, (0, 0, 0))


        approx = cv2.approxPolyDP(largest_contour, 0.01 * cv2.arcLength(largest_contour, True), True)

        if len(approx) == 6 or len(approx) == 7:
            print('cube detected.')

        #cv2.drawContours(original, contours, -1, (255, 0, 0))

        cv2.imshow('original', original)

        cv2.imshow('resized', edges) 

        key = cv2.waitKey(1)
        
        if key == 10: # Return key pressed
            print('Image captured')
            cv2.imwrite('image.jpg', resized)

    def get_largest_contour(self, contours):

        largest_contour_area = 0
        largest_contour = None
        for contour in contours:
            contour_area = cv2.contourArea(contour)
            if contour_area > largest_contour_area:
                largest_contour_area = contour_area
                largest_contour = contour

        return largest_contour

def main():
    print('Running... Press CTRL-C to quit.')
    ShapeRecognizer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Quitting')
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
