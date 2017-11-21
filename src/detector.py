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
        self.found_face = False
        self.faces = []
        self.currently_template_matching = False

    def loadFrameFindFace(self, frame):
        self.frame = frame
        
        self.frame = cv2.resize(self.frame, None, fx=self.SCALE_FACTOR, fy=self.SCALE_FACTOR, interpolation=cv2.INTER_AREA)
        self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        if not self.found_face:
            self.detect_all_faces(self.frame)
        else:
            self.detect_face_in_roi(self.frame)
            if self.currently_template_matching:
                self.detect_face_with_template_matching(self.frame)
        return self.face_pos
    
    def doubled_rectangle(self, input_rectangle, frame_size):
        """
        input_rectangle is a Rectangle()
        frame_size is a tuple (frame width, frame height)

        returns new rectangle doubled the size, centered around the same point.
        if the new rectangle is out of bounds, we maintain the width and height and
        bring it into the frame space.
        """
        new_rect = Rectangle()
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
        faces is an array of numpy matrices (a matrix of pixels)

        returns the face matrix with the biggest area = width * height
        """
        biggest_face = (None, 0)
        for f in faces:
            w, h = f.shape
            if biggest_face[1] < w*h:
                biggest_face = (f, w*h)
        return biggest_face[0]

    def get_rectangle_center(self, rectangle):
        """
        rectangle is a Rectangle()

        returns the center Point()
        """
        return Point(rectangle.top_left.x + rectangle.width / 2, \
                     rectangle.top_left.y + rectangle.height / 2)

    
    def get_face_template(self, frame, current_face):
        """
        frame is the global frame
        current_face is a Rectangle() describing the face we're considering

        returns the face's template matrix, which is the small patch rectangle
        in the middle of the face
        """
        new_face_rect = Rectangle()
        new_face_rect.top_left = Point(current_face.top_left.x + current_face.width /4, current_face.top_left.y + current_face.height / 4)
        new_face_rect.width = current_face.width / 2
        new_face_rect.height = current_face.height / 2

        return frame[new_face_rect.top_left.x:new_face_rect.top_left.x + new_face_rect.width, \
                     new_face_rect.top_left.y:new_face_rect.top_left.y + new_face_rect.height]

    def detect_all_faces(self, frame):
        

