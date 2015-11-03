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

        self.bridge = CvBridge()

        cv2.namedWindow('mask', cv2.WINDOW_NORMAL)
        cv2.createTrackbar('blur', 'mask', 3, 10, self.cb)
        cv2.createTrackbar('erosion', 'mask', 20, 50, self.cb)
        cv2.createTrackbar('dilation', 'mask', 1, 50, self.cb)

        #cv2.namedWindow('original', cv2.WINDOW_NORMAL)

        #cv2.namedWindow('mask', cv2.WINDOW_NORMAL)

        #cv2.namedWindow('smooth', cv2.WINDOW_NORMAL)
        #cv2.createTrackbar('inpaintRadius', 'smooth', 0, 10, self.cb)

        #cv2.namedWindow('edges', cv2.WINDOW_NORMAL)
        #cv2.createTrackbar('threshold1', 'edges', 0, 300, self.cb)
        #cv2.createTrackbar('threshold2', 'edges', 0, 300, self.cb)

        # OpenCV variables

        #self.scale = 1
        #self.pad = 20

    def cb(self, x):
        pass


    def get_largest_contours(self, contours):

        largest_contour_area = 0
        largest_contours = []
        for contour in contours:
            contour_area = cv2.contourArea(contour)
            if contour_area > largest_contour_area:
                largest_contours.append(contour)
                largest_contour_area = contour_area

        return largest_contours

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
        image = self.bridge.imgmsg_to_cv2(data)

        # Convert to numpy array
        image = np.array(image, dtype=np.float32)

        cv2.imshow('original', image)

        padx = 80
        pady = 20

        height, width = image.shape

        image = image[pady: height - pady, padx: width - padx]

        cv2.imshow('padded', image)

        # Normalize depth image to range [0, 255]
        cv2.normalize(image, image, 255, 0, cv2.NORM_MINMAX)

        # Convert to numpy array (unint8)
        image = np.array(image, dtype=np.uint8)

        # Threshold zero values and create mask
        #_, mask = cv2.threshold(image, 1, 255, type=cv2.THRESH_BINARY_INV)

        mask = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 3, 2)

        mask = self.process_mask(mask)

        cv2.imshow('mask', mask)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        largest_contours = self.get_largest_contours(contours)

        if len(largest_contours) > 1:
            x, y, w, h = cv2.boundingRect(largest_contours[-2])
            object_image = image[y:y + h, x: x + w]
            cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 255), 2)

        # #for i, contour in enumerate(contours):
        #cv2.drawContours(image, contours, -1, (255, 255, 255))

        # detector = cv2.SimpleBlobDetector()

        # keypoints = detector.detect(mask)

        # print(len(keypoints))

        # cv2.drawKeypoints(image, keypoints, np.array([]), (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        #cv2.imshow('contours', image)

        # # Scale image down
        # image = cv2.resize(image, (0, 0), fx=self.scale, fy=self.scale)

        

        

        cv2.imshow('contours', image)

        

        # cv2.imshow('mask', mask)

        # #edges = cv2.Canny(image, 100, 200)

        # inpaintRadius = cv2.getTrackbarPos('inpaintRadius', 'smooth')

        #smooth = cv2.inpaint(image, mask, 2, cv2.INPAINT_TELEA)

        #cv2.imshow('smooth', smooth)

        # #threshold1 = cv2.getTrackbarPos('threshold1', 'edges')
        # #threshold2 = cv2.getTrackbarPos('threshold2', 'edges')

        # #edges = cv2.Canny(smooth, threshold1, threshold2)

        # #cv2.imshow('edges', edges)

        cv2.waitKey(1)

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
