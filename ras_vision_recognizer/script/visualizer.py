#!/usr/bin/env python

# Future imports
from __future__ import print_function

# Regular imports
import sys
import numpy as np
import cv2
import signal

# ROS imports
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import UInt8
from ras_vision_recognizer.msg import Rect
from cv_bridge import CvBridge


class Visualizer:

    def __init__(self):

        self.node_name = 'color_recognizer'

        rospy.init_node(self.node_name, anonymous=True)

        self.image = None
        self.point = None

        self.subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.color_callback, queue_size=1)
        self.subscriber = rospy.Subscriber('/vision/object_rect', Point, self.point_callback, queue_size=1)

        self.publisher = rospy.Publisher('/object/color', UInt8, queue_size=1)

        self.bridge = CvBridge()

        cv2.namedWindow('visualization', cv2.WINDOW_NORMAL)
        cv2.createTrackbar('size', 'visualization', 0, 200, self.cb)


    def cb(self, x):
        pass

    def point_callback(self, point):

        self.point = point
        
        #cv2.circle(self.image, (px-s/2, py), 15, (255, 255, 255))

    def color_callback(self, data):
        
        # Convert from ROS image to OpenCV image
        self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        if self.point is not None:
            px = int(self.point.x)
            py = int(self.point.y)
            s = int(38.5 / self.point.z)
            cv2.rectangle(self.image, (px - s / 2, py - s / 2), (px + s / 2, py + s / 2), (0, 0, 0))

        cv2.imshow('visualization', self.image)

        if cv2.waitKey(1) == 27: # ESC
            shutdown()

def main():
    Visualizer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Quitting')
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
