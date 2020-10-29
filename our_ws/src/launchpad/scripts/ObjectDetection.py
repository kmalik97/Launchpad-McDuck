import time

import cv2
import argparse
import numpy as np



gimpH = 201
gimpS = 77
gimpV = 85
opencvH = gimpH / 2
opencvS = (gimpS / 100) * 255
opencvV = (gimpV / 100) * 255
lower_blue = np.array([100, 150, 0])
upper_blue = np.array([140, 255, 255])
lower_orange = np.array([14, 200, 230])
upper_orange = np.array([30, 260, 288])

#cap = cv2.VideoCapture('20201006_102258.mp4')
cap = cv2.VideoCapture(0)
time.sleep(3)

#Default
upper = upper_blue
lower = lower_blue

res_x = 4032
res_y = 1960

resImage = [4032, 1960]
resScreen = [2560, 1080]

_, frame = cap.read()

while(True):
    prev_frame = frame
    _, frame = cap.read()

    #convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    #Diliations and erosions to remove any small blobs left in image
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask,None,iterations=2)
    mask = cv2.dilate(mask,None,iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    c = max(cnts, key=cv2.contourArea)
    ((x, y), radius) = cv2.minEnclosingCircle(c)
    M = cv2.moments(c)


    if(radius > 5):
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        cv2.circle(frame,(int(x),int(y)), int(radius),(0,255,255),2)
        cv2.circle(frame,center,5,(0,0,255),-1)

    #print(center)

    res = cv2.bitwise_and(frame, frame, mask=mask)
    font = cv2.FONT_HERSHEY_SIMPLEX
    centerText = "Center Coordinates : ({x_coord} , {y_coord})".format(x_coord = center[0], y_coord=center[1])

    cv2.putText(frame, centerText, (4, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)



    cv2.imshow('image', frame)
    cv2.imshow('mask', mask)
    cv2.imshow('res', res)



    k = cv2.waitKey(5)
    if k == 27:
        break
    # elif k == ord('b'): #Blue
    #     print('Key Press K')
    #     lower = np.array([100, 150, 0])
    #     upper = np.array([140, 255, 255])
    # elif k == ord('o'):
    #     print('Key Press O')
    #     lower = np.array([14, 200, 230])
    #     upper = np.array([30, 260, 288])


    else:
        continue


cv2.destroyAllWindows()
# res_x = 4032
# res_y = 1960
# scaleFactor = 0.25;
# img = cv2.imread('rubixCube.jpg')
# img = cv2.resize(img,(int(res_x*scaleFactor),int(res_y*scaleFactor)))
# cv2.imshow('image',img)
# cv2.waitKey()

#
# lower_blue = np.array([100,150,0])
# upper_blue = np.array([140,255,255])
# mask = cv2.inRange(hsv, lower_blue, upper_blue)
# res = cv2.bitwise_and(img,img, mask= mask)
# cv2.imshow('image',img)
# cv2.imshow('mask',mask)
# cv2.imshow('res',res)
# cv2.waitKey()
