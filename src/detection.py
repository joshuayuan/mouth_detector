#!/usr/bin/env python
import cv2
import numpy as np


class Detector:
    class Point:
        def __init__(self, x=0, y=0):
            self.x = x
            self.y = y

    SCALE_FACTOR = 0.5
    def __init__(self):
        self.face_pos = Point()
        self.frame = None

    def findFace(frame):
        self.frame = frame
        
        self.frame = cv2.resize(self.frame, None, fx=self.SCALE_FACTOR, fy=self.SCALE_FACTOR, interpolation=cv2.INTER_AREA)


