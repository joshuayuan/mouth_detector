#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import detector
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class Node:

    def __init__(self):
        self.face_cascade= cv2.CascadeClassifier('./haarcascade_frontalface_default.xml')
        self.ds_factor = 0.5

        if self.face_cascade.empty():
            raise IOError('Unable to load the face cascade classifier xml file')

        self.bridge = CvBridge()
        self.frame = None
        self.detector = detector.Detection()


        rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)

    def callback(self, image):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self.detector.findFace(self.frame)
        
        if self.detector.faceIsFound():
            cv2.rectangle(self.frame, self.detector.


        face_rects = self.face_cascade.detectMultiScale(gray, 1.3, 10)
        for (x,y,w,h) in face_rects:
            cv2.rectangle(self.frame, (x,y), (x+w,y+h), (0,255,0), 3)
            break

        cv2.imshow('Face Detector', self.frame)
        cv2.waitKey(1)


if __name__ == '__main__':
    node = Node()
    rospy.init_node('node')
    rospy.spin()