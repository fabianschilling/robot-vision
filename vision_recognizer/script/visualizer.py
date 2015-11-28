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
from vision_msgs.msg import Detection
from cv_bridge import CvBridge


class Visualizer:

    def __init__(self):

        self.node_name = 'color_recognizer'

        rospy.init_node(self.node_name, anonymous=True)

        self.image = None
        self.detection = None

        self.subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.color_callback, queue_size=1)
        self.subscriber = rospy.Subscriber('/vision/detection', Detection, self.detection_callback, queue_size=1)

        self.bridge = CvBridge()

        cv2.namedWindow('visualization', cv2.WINDOW_NORMAL)
        cv2.createTrackbar('size', 'visualization', 0, 200, self.cb)


    def cb(self, x):
        pass

    def detection_callback(self, message):

        self.detection = message
        
        #cv2.circle(self.image, (px-s/2, py), 15, (255, 255, 255))

    def color_callback(self, message):
        
        # Convert from ROS image to OpenCV image
        self.image = self.bridge.imgmsg_to_cv2(message, 'bgr8')

        if self.detection is not None:
            box = self.detection.box
            ctr = self.detection.centroid
            cv2.circle(self.image, (int(ctr.x), int(ctr.y)), 3, (255, 255, 255))
            cv2.rectangle(self.image, (box.x, box.y), (box.x + box.size, box.y + box.size), (0, 0, 0))

        # if self.point is not None:

        #     #for pt in self.point.points:
        #     #    px = int(pt.x)
        #     #    py = int(pt.y)
        #     #    cv2.circle(self.image, (px, py), 3, (255, 255, 255))
        #     px = int(self.point.x)
        #     py = int(self.point.y)
        #     s = int(38.5 / self.point.z)
        #     cv2.circle(self.image, (px, py), 3, (255, 255, 255))
        #     cv2.rectangle(self.image, (px - s / 2, py - s / 2), (px + s / 2, py + s / 2), (0, 0, 0))

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
