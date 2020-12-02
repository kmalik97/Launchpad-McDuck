import cv2
import numpy as np
import time

cap = cv2.VideoCapture(0)
time.sleep(3)


# Red values
lower = np.array([0, 180, 52])
upper = np.array([17, 210, 200])


while(True):
    _, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower, upper)
    #mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)

    output = cv2.bitwise_and(frame,frame,mask=mask)

    cnts = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    if not cnts:
        print("No Red Found")
        #Return Statement here
    else:

        c = max(cnts, key=cv2.contourArea)
        threshold = 20

        if cv2.contourArea(c) < threshold:
            print("Area under threshold")
        else:
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            font = cv2.FONT_HERSHEY_SIMPLEX
            centerText = "Center Coordinates : ({x_coord} , {y_coord})".format(x_coord=center[0], y_coord=center[1])
            cv2.putText(frame, centerText, (4, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.imshow("images",mask)
    cv2.imshow("img",output)
    cv2.imshow("org", frame)
    k = cv2.waitKey(5)
    if k == 27:
        break



cv2.destroyAllWindows()