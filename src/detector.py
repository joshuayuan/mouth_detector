#!/usr/bin/env python
import cv2
import numpy as np


class Detector:
    class Point:
        def __init__(self, x=0, y=0):
            self.x = x # int
            self.y = y # int
    class Rectangle:
        def __init__(self, top_left=Point(), width=0, height=0):
            self.top_left = top_left # Point()
            self.width = width # int
            self.height = height # int

    SCALE_FACTOR = 0.5
    def __init__(self):
        self.face_pos = Point()
        self.frame = None

    def findFace(frame):
        self.frame = frame
        
        self.frame = cv2.resize(self.frame, None, fx=self.SCALE_FACTOR, fy=self.SCALE_FACTOR, interpolation=cv2.INTER_AREA)


        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
