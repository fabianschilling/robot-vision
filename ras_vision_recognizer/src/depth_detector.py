#!/usr/bin/env python

# Regular imports
import sys
import numpy as np
import cv2

# ROS imports
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DepthDetector:

    def __init__(self):

        self.node_name = 'depth_detector'

        rospy.init_node(self.node_name, anonymous=True)

        self.subscriber = rospy.Subscriber('camera/depth/image', Image, self.depth_callback, queue_size=1)
        self.subscriber = rospy.Subscriber('camera/rgb/image_raw', Image, self.color_callback, queue_size=1)

        self.bridge = CvBridge()

        cv2.namedWindow('mask', cv2.WINDOW_NORMAL)
        cv2.createTrackbar('blur', 'mask', 3, 10, self.cb)
        cv2.createTrackbar('erosion', 'mask', 20, 50, self.cb)
        cv2.createTrackbar('dilation', 'mask', 1, 50, self.cb)

        cv2.namedWindow('thresh', cv2.WINDOW_NORMAL)
        cv2.createTrackbar('hue_min', 'thresh', 0, 179, self.cb)
        cv2.createTrackbar('hue_max', 'thresh', 179, 179, self.cb)
        cv2.createTrackbar('sat_min', 'thresh', 150, 255, self.cb)
        cv2.createTrackbar('sat_max', 'thresh', 255, 255, self.cb)
        cv2.createTrackbar('val_min', 'thresh', 70, 255, self.cb)
        cv2.createTrackbar('val_max', 'thresh', 255, 255, self.cb)

        # OpenCV variables

        #self.scale = 1
        #self.pad = 20
        self.object_contour = None

    def cb(self, x):
        pass

    def color_callback(self, data):

        # Don't go further if no object is recognized
        if self.object_contour is None:
            return

        # Convert from ROS image to OpenCV image
        original = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        # Unwrap object contour
        x, y, w, h = self.object_contour

        # Crop image
        image = original[y:y + h, x: x + w]

        # Do color thresholding
        thresh, mask = self.color_threshold(image)
        
        cv2.imshow('thresh', thresh)
        #cv2.imshow('objmask', mask)
        cv2.imshow('object', image)

    def color_threshold(self, image):

        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        hue_min = cv2.getTrackbarPos('hue_min', 'thresh')
        hue_max = cv2.getTrackbarPos('hue_max', 'thresh')
        sat_min = cv2.getTrackbarPos('sat_min', 'thresh')
        sat_max = cv2.getTrackbarPos('sat_max', 'thresh')
        val_min = cv2.getTrackbarPos('val_min', 'thresh')
        val_max = cv2.getTrackbarPos('val_max', 'thresh')


        lower = np.array([hue_min, sat_min, val_min])
        upper = np.array([hue_max, sat_max, val_max])

        mask = cv2.inRange(hsv_image, lower, upper)

        thresh = cv2.bitwise_and(image, image, mask=mask)

        return (thresh, mask)

    def process_mask(self, mask):

        blur = cv2.getTrackbarPos('blur', 'processed')

        if blur > 0:
            image = cv2.blur(image, (blur, blur))

        erosion = cv2.getTrackbarPos('erosion', 'mask')

        if erosion > 0:
            mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (erosion, erosion)))

        dilation = cv2.getTrackbarPos('dilation', 'mask')

        if dilation > 0:
            mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilation, dilation)))

        return mask

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
        mask = self.process_mask(mask)

        # Show the filtered mask
        cv2.imshow('mask', mask)

        # Find contours in the mask
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        object_contour = self.detect_object(contours)

        if object_contour is not None:
            x, y, w, h = object_contour

            x += padx
            y += pady

            self.object_contour = (x, y, w, h)

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
