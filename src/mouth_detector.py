#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class MouthDetector:

    def __init__(self):
        self.mouth_cascade = cv2.CascadeClassifier('haarcascade_mcs_mouth.xml')
        self.ds_factor = 0.5

        if self.mouth_cascade.empty():
            raise IOError('Unable to load the mouth cascade classifier xml file')

        self.bridge = CvBridge()
        self.frame = None
        rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)


    def callback(self, image):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.frame = cv2.resize(self.frame, None, fx=self.ds_factor, fy=self.ds_factor, interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        mouth_rects = self.mouth_cascade.detectMultiScale(gray, 1.7, 11)
        for (x,y,w,h) in mouth_rects:
            y = int(y - 0.15*h)
            cv2.rectangle(self.frame, (x,y), (x+w,y+h), (0,255,0), 3)
            break

        cv2.imshow('Mouth Detector', self.frame)
        cv2.waitKey(1)


if __name__ == '__main__':
    node = MouthDetector()
    rospy.init_node('mouth_detector')
    rospy.spin()
