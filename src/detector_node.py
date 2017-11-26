#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import rospkg
import detector
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class Node:

    SCALE_FACTOR = 0.5
    def __init__(self):
        self.bridge = CvBridge()
        self.detector = detector.Detector()
        # import pdb; pdb.set_trace()
        rospack = rospkg.RosPack()
        cascade_path = rospack.get_path('mouth_detector')+"/src/haarcascade_frontalface_default.xml"
        #cascade_path = rospy.get_param('/detector_node/cascade_path')
        print(cascade_path)
        self.detector.loadCascade(cascade_path)
        if self.detector.hasEmptyCascade():
            raise IOError('Unable to load the face cascade classifier xml file')

        rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)

        self.basic_cascade = cv2.CascadeClassifier(cascade_path)
        self.basic_frame = None
        self.prev_coords = [(0, 0),(1, 1)] # (x, y) of top left
        self.prev_dimensions = (0,0)
        self.loaded = False

    def callback(self, image):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.detector.loadFrameFindFace(self.frame)

        #self.frame = cv2.resize(self.frame, None, fx=self.SCALE_FACTOR, fy=self.SCALE_FACTOR, interpolation=cv2.INTER_AREA)
        #self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        #basic_faces = self.basic_cascade.detectMultiScale(self.frame, 1.2, 3)
        #if len(basic_faces) != 0:
        #    x1, y1, x2, y2 = basic_faces[0]
        #    cv2.rectangle(self.frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

        if self.detector.faceIsFound():
            x1, y1, x2, y2 = self.detector.getFaceBox()
            px1, py1 = self.prev_coords[0]
            px2, py2 = self.prev_coords[1]
            print("-", x1, y1, x2, y2)
            print("P", px1, py1, px2, py2)
            if self.loaded and ( abs(x1-px1) > self.frame.shape[0] / 8 or abs(y1-py1) / 8 > self.frame.shape[1]):
                x1, y1 = self.prev_coords[0]
                x2, y2 = self.prev_coords[1]
            elif self.loaded:
                self.prev_coords[0] = (x1, y1)
                self.prev_coords[1] = (x2, y2)
            if self.loaded and ((x2-x1)/2 > (px2-px1) or (y2-y1)/2 > (py2-py1) or (x2-x1)*2 < (px2-px1) or (y2-y1)*2 < (py2-py1)):
                #print((x2-x1)/2 > (px2-px1) or (y2-y1)/2 > (py2-py1) or (x2-x1)*2 < (px2-px1) or (y2-y1)*2 < (py2-py1))
                cv2.rectangle(self.frame, (x1, y1), (x2, y2), (0, 255, 0), 4)
                x1, y1 = self.prev_coords[0]
                x2, y2 = self.prev_coords[1]
            else:
                self.prev_coords[0] = (x1, y1)
                self.prev_coords[1] = (x2, y2)
            cv2.rectangle(self.frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
            cv2.circle(self.frame, ((x1+x2)/2, y1/5+4*y2/5), (x2-x1)/6, (0, 0, 255), 2)

        cv2.imshow('Face Detector', self.frame)
        key = cv2.waitKey(1)
        if key == ord('l'):
            self.loaded = True
        elif key == ord('u'):
            self.loaded = False
        elif key == ord('q'):
            rospy.signal_shutdown("q was pressed")


if __name__ == '__main__':
    node = Node()
    rospy.init_node('node')
    rospy.spin()
