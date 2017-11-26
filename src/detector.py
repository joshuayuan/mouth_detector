#!/usr/bin/env python
import cv2
import numpy as np
import time


class Detector:
    class Point:
        def __init__(self, x=0, y=0):
            self.x = x # int
            self.y = y # int
    class Rectangle:
        def __init__(self, top_left=None, width=0, height=0):
            if top_left == None:
                top_left = Detector.Point()

            self.top_left = top_left # Point()
            self.width = width # int
            self.height = height # int
    class Timer:
        def __init__(self):
            self.start = 0
            self.runnig = False

    SCALE_FACTOR = 0.5
    def __init__(self):
        self.face_pos = self.Point()
        self.face = self.Rectangle()
        self.frame = None
        self.found_face = False
        self.faces = []
        self.face_roi = self.Rectangle()
        self.face_template = None # a np matrix
        self.match_result = None # a np matrix i think
        self.timer = self.Timer()

        self.face_cascade = None
        self.i = 1

    def loadCascade(self, path):
        self.face_cascade = cv2.CascadeClassifier(path)
        print(self.face_cascade)
        print(path)

    def hasEmptyCascade(self):
        return self.face_cascade.empty()

    def loadFrameFindFace(self, frame):
        self.frame = frame
        if self.i == 1:
            print(self.frame.shape)
            print(self.frame)
            self.i = 2

        self.frame = cv2.resize(self.frame, None, fx=self.SCALE_FACTOR, fy=self.SCALE_FACTOR, interpolation=cv2.INTER_AREA)
        self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        if not self.found_face:
            self.detect_all_faces(self.frame)
        else:
            self.detect_faces_around_roi(self.frame)
            if self.timer.running:
                self.detect_faces_with_template_matching(self.frame)
        return self.face_pos
    
    def double_this_rectangle(self, input_rectangle, frame_size):
        """
        input_rectangle is a Rectangle()
        frame_size is a tuple (frame width, frame height)

        returns new rectangle doubled the size, centered around the same point.
        if the new rectangle is out of bounds, we maintain the width and height and
        bring it into the frame space.
        """
        new_rect = self.Rectangle()
        new_rect.width = input_rectangle.width * 2
        new_rect.height = input_rectangle.height * 2
        new_rect.top_left.x = max(0, input_rectangle.top_left.x - input_rectangle.width / 2)
        new_rect.top_left.y = max(0, input_rectangle.top_left.y - input_rectangle.height / 2)

        if new_rect.width + new_rect.top_left.x > frame_size[0]:
            new_rect.top_left.x = frame_size[0] - new_rect.width + 1
        if new_rect.height + new_rect.top_left.y > frame_size[1]:
            new_rect.top_left.y = frame_size[1] - new_rect.height + 1
        return new_rect

    def find_biggest_face(self, faces):
        """
        faces is an array of rectangles of the format (x, y, w, h).

        returns the Rectangle() with the biggest area.
        """
        biggest_face = (None, 0)
        for x, y, w, h in faces:
            if biggest_face[1] < w*h:
                biggest_face = (self.Rectangle(self.Point(x, y), w, h), w*h)
        return biggest_face[0]

    def get_rectangle_center(self, rectangle):
        """
        rectangle is a Rectangle()

        returns the center Point() of that rectangle
        """
        return self.Point(rectangle.top_left.x + rectangle.width / 2, \
                     rectangle.top_left.y + rectangle.height / 2)

    
    def get_face_template(self, frame, current_face):
        """
        frame is the global frame, a numpy matrix
        current_face is a Rectangle() describing the area of the frame we're considering

        returns the face template as a np matrix, which is the small patch rectangle
        in the middle of the face
        """
        new_face_rect = self.Rectangle()
        new_face_rect.top_left = self.Point(current_face.top_left.x + current_face.width /4, current_face.top_left.y + current_face.height / 4)
        new_face_rect.width = current_face.width / 2
        new_face_rect.height = current_face.height / 2

        return frame[new_face_rect.top_left.x:new_face_rect.top_left.x + new_face_rect.width, \
                     new_face_rect.top_left.y:new_face_rect.top_left.y + new_face_rect.height]

    def detect_all_faces(self, frame):
        self.faces = self.face_cascade.detectMultiScale(self.frame, 1.1, 5) #Add min & max sizing later
        if not len(self.faces):
            print("no faces")
            return

        print("found a face")
        self.found_face = True
        self.face = self.find_biggest_face(self.faces)
        self.face_template = self.get_face_template(self.frame, self.face)
        self.face_roi = self.double_this_rectangle(self.face, self.frame.shape)
        self.face_pos = self.get_rectangle_center(self.face)

    def detect_faces_around_roi(self, frame):
        self.faces = self.face_cascade.detectMultiScale(self.frame, 1.1, 5) #min and max should be +/- 20%
        if len(self.faces) == 0:
            self.timer.running = True
            if self.timer.start == 0:
                self.timer.start = time.time()
            return
        self.timer.running = False
        self.timer.start = 0
        
        self.face = self.find_biggest_face(self.faces)

        self.face_template = self.get_face_template(self.frame, self.face)
        self.face_roi = self.double_this_rectangle(self.face, self.frame.shape)
        self.face_pos = self.get_rectangle_center(self.face)

    def detect_faces_with_template_matching(self, frame):
        if time.time() - self.timer.start > 2:
            self.found_face = False
            self.timer.running = False
            self.timer.start = 0
            self.face_pos = self.Point()
            self.face = self.Rectangle()
            return
        self.match_result = cv2.matchTemplate(frame, self.face_template, cv2.TM_SQDIFF_NORMED)
        cv2.normalize(self.match_result, self.match_result, 0, 1, cv2.NORM_MINMAX)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(self.match_result)
        top_left = max_loc
        bottom_right = (top_left[0] + self.face_template.shape[0], top_left[1] + self.face_template.shape[1])
        
        self.face = self.Rectangle(self.Point(min_loc[0], min_loc[1]), self.face_template.shape[0], self.face_template.shape[1])
        self.face = self.double_this_rectangle(self.face, self.frame.shape)

        self.face_template = self.get_face_template(self.frame, self.face)
        self.face_roi = self.double_this_rectangle(self.face, self.frame.shape)
        self.face_pos = self.get_rectangle_center(self.face)
    def faceIsFound(self):
        return self.found_face
    def getFacePosition(self):
        return self.face_pos
    def getFaceBox(self):
        return [int(x / self.SCALE_FACTOR) for x in (self.face.top_left.x, self.face.top_left.y, self.face.top_left.x + self.face.width, self.face.top_left.y + self.face.height)]

    def getMouthPos(self):
        return (self.face_pos.x, self.face_pos.y + self.face.height * 4/5)
