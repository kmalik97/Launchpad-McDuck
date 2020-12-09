import cv2
import numpy as np
import time
cap = cv2.VideoCapture(0)
time.sleep(3)
count=0
while(True):
    _, frame = cap.read()
    #cv2.imshow('image', frame)
    k = cv2.waitKey(0)
    if k == 27:
        cv2.imwrite('../images/frame_%d.jpg'% count, frame)
        count = count + 1
cv2.destroyAllWindows()
