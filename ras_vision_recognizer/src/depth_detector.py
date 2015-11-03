#!/usr/bin/env python

# Regular imports
import sys
import numpy as np
import cv2

# ROS imports
import rospy
from sensor_msgs.msg import Image
from ras_vision_recognizer.msg import Rect
from cv_bridge import CvBridge


class DepthDetector:

    def __init__(self):

        self.node_name = 'depth_detector'

        rospy.init_node(self.node_name, anonymous=True)

        self.subscriber = rospy.Subscriber('camera/depth/image', Image, self.depth_callback, queue_size=1)

        self.publisher = rospy.Publisher('vision/object_rect', Rect, queue_size=1)

        self.bridge = CvBridge()

        self.erosion = 20
        self.dilation = 1

        self.object_contour = None

    def cb(self, x):
        pass

    def depth_callback(self, data):

        # Convert from ROS image to OpenCV image
        original = self.bridge.imgmsg_to_cv2(data)

        # Convert to numpy array
        original = np.array(original, dtype=np.float32)

        # Normalize depth image to range [0, 255]
        cv2.normalize(original, original, 255, 0, cv2.NORM_MINMAX)

        # Convert to numpy array (unint8)
        original = np.array(original, dtype=np.uint8)

        # Crop out part of the image that does not have noise
        padx = 80
        pady = 20
        height, width = original.shape
        image = original[pady: height - pady, padx: width - padx]
        cv2.rectangle(original, (padx, pady), (width - padx, height - pady), (255, 255, 255), 1)

        # Adaptive threshold detects NaN blobs and gradient changes
        mask = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 3, 2)

        # Apply erosion and dilation
        #mask = self.process_mask(mask)
        mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.erosion, self.erosion)))
        mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.dilation, self.dilation)))

        # Show the filtered mask
        #cv2.imshow('mask', mask)

        # Find contours in the mask
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        object_contour = self.detect_object(contours)

        if object_contour is not None:
            x, y, w, h = object_contour

            x += padx
            y += pady

            self.object_contour = (x, y, w, h)

            msg = Rect()
            msg.x = x
            msg.y = y
            msg.width = w
            msg.height = h

            self.publisher.publish(msg)

            # Mind the padding!
            cv2.rectangle(original, (x, y), (x + w, y + h), (255, 255, 255), 1) 

        cv2.imshow('detection', original)

        cv2.waitKey(1)

    def detect_object(self, contours):

        # Get (second) largest contour and 
        largest_contour_area = 0
        largest_contours = []
        for contour in contours:
            contour_area = cv2.contourArea(contour)
            if contour_area > largest_contour_area:
                largest_contours.append(contour)
                largest_contour_area = contour_area

        if len(largest_contours) > 1:

            # Get bounding rectangle of largest object
            x, y, w, h = cv2.boundingRect(largest_contours[-2])

            # Get size and aspect ratio of the detected object
            size = w * h
            aspect = 1.0 * w / h

            # Decide if this is object based on size and aspect ratio
            if size > 5000 and size < 14000 and aspect > 0.9 and aspect < 1.1:
 
                return (x, y, w, h)

        return None 


def main():
    print('Running... Press CTRL-C to quit.')
    DepthDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Quitting')
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
