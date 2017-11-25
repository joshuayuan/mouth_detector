import cv2
import numpy as np
import detector

mouth_cascade = cv2.CascadeClassifier('/mnt/c/Berkeley/ee106a/mouth_detector/src/haarcascade_frontalface_default.xml')

if mouth_cascade.empty():
    raise IOError('Unable to load the mouth cascade classifier xml file')

cap = cv2.VideoCapture(0)
ds_factor = 0.5
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

cap.release()
cv2.destroyAllWindows()
