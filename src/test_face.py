import cv2
import numpy as np
<<<<<<< HEAD

mouth_cascade = cv2.CascadeClassifier('./haarcascade_mcs_mouth.xml')
=======
import detector

mouth_cascade = cv2.CascadeClassifier('/mnt/c/Berkeley/ee106a/mouth_detector/src/haarcascade_frontalface_default.xml')
>>>>>>> edbd127b8a02a424e5b2e4d20a488e92b5e1482f

if mouth_cascade.empty():
    raise IOError('Unable to load the mouth cascade classifier xml file')

cap = cv2.VideoCapture(0)
ds_factor = 0.5
<<<<<<< HEAD

while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame, None, fx=ds_factor, fy=ds_factor, interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    mouth_rects = mouth_cascade.detectMultiScale(gray, 1.7, 11)
    for (x,y,w,h) in mouth_rects:
        y = int(y - 0.15*h)
        cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 3)
        break

    cv2.imshow('Mouth Detector', frame)

    c = cv2.waitKey(1)
    if c == 27:
        break
=======
detector = detector.Detector()
detector.loadCascade('./haarcascade_frontalface_default.xml')
while True:
    ret, frame = cap.read()
    print(frame)
    detector.loadFrameFindFace(frame)
    if detector.faceIsFound():
        x1, y1, x2, y2 = detector.getFaceBox()
        print(x1, y1, x2, y2)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)

    cv2.imshow('Face Detector', frame)
    cv2.waitKey(1)
>>>>>>> edbd127b8a02a424e5b2e4d20a488e92b5e1482f

cap.release()
cv2.destroyAllWindows()
