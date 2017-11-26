#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import detector
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class Node:
    SCALE_FACTOR = 0.5
    def __init__(self):
        self.bridge = CvBridge()
        self.detector = detector.Detector()
        print("PLEASE")
        cascade_path = rospy.getparam('cascade_path')
        self.detector.loadCascade(cascade_path)
        if self.detector.hasEmptyCascade():
            raise IOError('Unable to load the face cascade classifier xml file')

        rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)

        self.basic_cascade = cv2.CascadeClassifier(path)
        self.basic_frame = None
        self.previous_coords = ((0, 0),(1, 1)) # (x, y) of top left

    def callback(self, image):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.detector.loadFrameFindFace(self.frame)

        self.basic_frame = cv2.resize(self.basic_frame, None, fx=self.SCALE_FACTOR, fy=self.SCALE_FACTOR, interpolation=cv2.INTER_AREA)
        self.basic_frame = cv2.cvtColor(self.basic_frame, cv2.COLOR_BGR2GRAY)
        basic_faces = self.basic_cascade.detectMultiScale(self.basic_frame, 1.2, 3)
        if len(basic_faces) != 0:
            x1, y1, x2, y2 = basic_faces[0]
            cv2.rectangle(self.frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

        if self.detector.faceIsFound():
            x1, y1, x2, y2 = self.detector.getFaceBox()
            print(x1, y1, x2, y2)
            if abs(x1-self.prev_coords[0][0]) > self.frame.shape[0] / 4 or abs(y1-self.prev_coords[0][1]) > self.frame.shape[1]:
                x1, y1 = self.prev_coords[0]
                x2, y2 = self.prev_coords[1]
            else:
                self.prev_coords[0] = (x1, y1)
                self.prev_coords[1] = (x2, y2)
            cv2.rectangle(self.frame, (x1, y1), (x2, y2), (255, 0, 255), 2)

        cv2.imshow('Face Detector', self.frame)
        cv2.waitKey(1)


if __name__ == '__main__':
    node = Node()
    rospy.init_node('node')
    rospy.spin()
