#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import detector
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class Node:

    def __init__(self):
        self.bridge = CvBridge()
        self.detector = detector.Detector()
        print("PLEASE")
        #  f = open(path)
        #  print f
        #  return
        self.detector.loadCascade('/home/brent/catkin_ws/src/mouth_detector/src/haarcascade_frontalface_default.xml')
        if self.detector.hasEmptyCascade():
            raise IOError('Unable to load the face cascade classifier xml file')

        rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)

    def callback(self, image):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.detector.loadFrameFindFace(self.frame)
        if self.detector.faceIsFound():
            x1, y1, x2, y2 = self.detector.getFaceBox()
            print(x1, y1, x2, y2)
            cv2.rectangle(self.frame, (x1, y1), (x2, y2), (255, 0, 255), 2)

        cv2.imshow('Face Detector', self.frame)
        cv2.waitKey(1)


if __name__ == '__main__':
    node = Node()
    rospy.init_node('node')
    rospy.spin()
