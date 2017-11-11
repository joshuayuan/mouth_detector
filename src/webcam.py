import numpy as np
import cv2

cap = cv2.VideoCapture(0)
minthresh = 100
maxthresh = 200
while True:
    ret, frame = cap.read()

    width, height, channels = frame.shape
    cv2.imshow('frame', frame)
    edges = cv2.Canny(frame, minthresh, maxthresh)
    cv2.putText(edges, "min: " + str(minthresh), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (200, 0, 0))
    cv2.putText(edges, "max: " + str(maxthresh), (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (200, 0, 0))
    cv2.imshow('edges', edges)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('a'):
        minthresh -= 10
    elif key == ord('s'):
        minthresh += 10
    elif key == ord('d'):
        maxthresh -= 10
    elif key == ord('f'):
        maxthresh += 10

cap.release()
cv2.destroyAllWindows()
