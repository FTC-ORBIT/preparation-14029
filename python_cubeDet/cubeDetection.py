import cv2
import numpy as np
from time import sleep

cap = cv2.VideoCapture(0)


# Trackbar
def nothing(x):
    pass


cv2.namedWindow('controls')
cv2.createTrackbar('r', 'controls', 0, 500, nothing)
cv2.createTrackbar('z', 'controls', 0, 500, nothing)

while True:
    # Reading the image
    ret, frame = cap.read()

    lowT = int(cv2.getTrackbarPos('r', 'controls'))
    highT = int(cv2.getTrackbarPos('z', 'controls'))
    frame = cv2.resize(frame, (320, 240), cv2.INTER_AREA)

    # Finding the edges

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    kernel = np.ones((5, 5), np.uint8)
    edged = cv2.Canny(gray, lowT, highT)

    dilation = cv2.dilate(edged, kernel, 1)

    # Finding the contours
    econtours, heir = cv2.findContours(dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # Checking if the object is a rectangle and drawing on it
    cx = 0
    cy = 0
    cw = 0
    ch = 0
    ex = 0
    ey = 0
    ew = 0
    eh = 0
    
    for econtour in econtours:
        approx = cv2.approxPolyDP(econtour, 0.01 * cv2.arcLength(econtour, True), True)
        (ex, ey, ew, eh) = cv2.boundingRect(econtour)

        # Referng to a specific area in the frame
        croppedFrame = frame[ex:ew, ey:eh]
        

        #Finding Color in the cropped frame
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #low = np.array([33, 99, 46])
        #high = np.array([43, 101, 46])
        low = np.array([21, 29, 68])
        high = np.array([22, 35, 71])

        mask = cv2.inRange(hsv, low, high)
        dilated = cv2.dilate(mask, None, 3)
        ccontours, heir = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for ccontour in ccontours:
            area = cv2.contourArea(ccontour)
            (cx, cy, cw, ch) = cv2.boundingRect(econtour)

       
        
        if 0 <= len(approx) <= 10:
            cx = int(cx + cw / 2)
            cy = int(cy + ch / 2)
            cv2.rectangle(frame, (cx, cy), (cx + cw, cy + ch), (255, 0, 0), 2)
            print(len(approx))
        else:
            pass

    # Showing the image
    cv2.imshow("original", frame)
    cv2.imshow("color", mask)
    cv2.imshow("edges", dilation)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
cv2.destroyAllWindows()
cap.release
