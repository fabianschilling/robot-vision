#!/usr/bin/env python

# Future imports
from __future__ import print_function

# Regular imports
import sys
import signal
import numpy as np
import cv2
import os

# ROS imports
import rospy
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image
from ras_vision_recognizer.msg import Rect
from cv_bridge import CvBridge


class MaterialRecognizer:

    def __init__(self):

        signal.signal(signal.SIGINT, signal_callback)

        self.node_name = 'material_recognizer'

        cv2.namedWindow('material', cv2.WINDOW_NORMAL)

        self.bridge = CvBridge()

        rospy.init_node(self.node_name, anonymous=True)

        self.image = None

        self.count = 0

        self.directory = '/home/fabian/catkin_ws/src/ras_vision/ras_vision_data/material/'

        self.depth_subscriber = rospy.Subscriber('/camera/depth/image', Image, self.depth_callback, queue_size=1)
        self.rect_subscriber = rospy.Subscriber('/vision/object_rect', Rect, self.object_callback, queue_size=1)

        self.publisher = rospy.Publisher('/object/material', UInt8, queue_size=1)


    def cb(self, x):
        pass

    def depth_callback(self, data):

        self.image = self.bridge.imgmsg_to_cv2(data)

    def object_callback(self, d):

        # Return if no image available
        if self.image is None:
            return

        # Crop the object
        image = self.image[d.y:d.y + d.height, d.x:d.x + d.width]

        cv2.imshow('material', image)

        height, width = image.shape
        size = height * width
        nans = np.count_nonzero(~np.isnan(image))

        nan_ratio = float(nans) / float(size)

        distance = np.nanmean(image)

        material = 'wood' if nan_ratio > 0.75 else 'plastic'

        msg = UInt8()
        msg.data = 0 if material == 'wood' else 1
        self.publisher.publish(msg)

        print('nan ratio: ' + str(nan_ratio))
        print('distance: ' + str(distance))

        key = cv2.waitKey(1)
        
        if key == 27: # ESC
            shutdown()
        elif key == 10:
            filename = self.directory + 'plastic' + str(self.count + 1) + '.jpg' 
            cv2.imwrite(filename, image)
            print('Captured image: ' + filename)
            self.count += 1

    def classify(self, image):

        data = np.array(image, dtype=np.float32)
        flattened = data.flatten()
        x = scale(flattened)
        predition = self.clf.predict(x)
        if predition == 0:
            print('cube')
        elif predition == 1:
            print('sphere')
        #print(x)

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
    MaterialRecognizer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Quitting')
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
