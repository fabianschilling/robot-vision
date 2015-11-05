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
from std_msgs.msg import UInt8
from ras_vision_recognizer.msg import Rect
from cv_bridge import CvBridge


class ColorRecognizer:

    def __init__(self):

        signal.signal(signal.SIGINT, signal_callback)

        self.node_name = 'color_recognizer'

        rospy.init_node(self.node_name, anonymous=True)

        self.image = None

        self.subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.color_callback, queue_size=1)
        self.subscriber = rospy.Subscriber('/vision/object_rect', Rect, self.object_callback, queue_size=1)

        self.publisher = rospy.Publisher('/object/color', UInt8, queue_size=1)

        self.bridge = CvBridge()

        cv2.namedWindow('thresh', cv2.WINDOW_NORMAL)
        cv2.createTrackbar('hue_min', 'thresh', 0, 179, self.cb)
        cv2.createTrackbar('hue_max', 'thresh', 179, 179, self.cb)
        cv2.createTrackbar('sat_min', 'thresh', 130, 255, self.cb)
        cv2.createTrackbar('sat_max', 'thresh', 255, 255, self.cb)
        cv2.createTrackbar('val_max', 'thresh', 255, 255, self.cb)

        # cv2.namedWindow('gray', cv2.WINDOW_NORMAL)

        # cv2.namedWindow('processed', cv2.WINDOW_NORMAL)
        # cv2.createTrackbar('blur', 'processed', 3, 30, self.cb)

        # cv2.namedWindow('edges', cv2.WINDOW_NORMAL)
        # cv2.createTrackbar('kernel', 'edges', 0, 30, self.cb)

    def cb(self, x):
        pass

    def object_callback(self, data):

        # Return if no image available
        if self.image is None:
            return

        # Crop the object
        image = self.image[data.y:data.y + data.height, data.x:data.x + data.width]

        #cv2.imshow('object', image)

        thresh, mask = self.color_threshold(image)
        cv2.imshow('thresh', thresh)

        color, color_name, prob = self.classify_color(thresh)

        if prob > 0.7 and prob is not None:
            print('Color: ' + str(color) + ', prob: ' + str(int(prob * 100)) + ' %', end='\r')
            sys.stdout.flush()       

            msg = UInt8()
            msg.data = color
            self.publisher.publish(msg)


        #gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        #msg = self.bridge.cv2_to_imgmsg(gray, 'mono8')

        #self.publisher.publish(msg)

        #shape = self.classify_shape(image)

        if cv2.waitKey(1) == 27: # ESC
            shutdown()

    def classify_color(self, object_image):

        #cv2.imshow('object', object_image)

        hsv_image = cv2.cvtColor(object_image, cv2.COLOR_BGR2HSV)

        hist = cv2.calcHist([hsv_image], [0], None, [180], [1, 179])

        total = float(np.sum(hist))

        if total == 0:
            return (None, None, None)

        orange = np.sum(hist[5:12])
        yellow = np.sum(hist[12:31])
        green = np.sum(hist[31:70])
        blue = np.sum(hist[70:120])
        purple = np.sum(hist[120:162])
        red = np.sum(hist[162:179]) + np.sum(hist[0:5])

        colors = [orange, yellow, green, blue, purple, red]
        #color_names = ['orange', 'yellow', 'green', 'blue', 'purple', 'red']
        color_names = ['o', 'y', 'g', 'b', 'p', 'r']
        color_hues = [6, 22, 50, 95, 140, 170]
        color_rgb = [(255, 255/2, 0), (255, 255, 0), (0, 255, 0), (0, 0, 255), (255, 0, 255), (255, 0, 0)]
        color_bgr = [(0, 255/2, 255), (0, 255, 255), (0, 255, 0), (255, 0, 0), (255, 0, 255), (0, 0, 255)]

        color = int(np.argmax(colors))

        probability = float(colors[color]) / total

        #print('Color: ' + str(color_names[color]) + ' (' + str(probability * 100) + ')')

        return (color, color_names[color], probability)

    def color_callback(self, data):
        
        # Convert from ROS image to OpenCV image
        self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

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
    ColorRecognizer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Quitting')
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
