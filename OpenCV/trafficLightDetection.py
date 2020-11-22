# Ioannis Broumas
# ioabro17@student.hh.se
# Nov 2020
# Traffic light detection

from __future__ import print_function
import cv2 as cv
import numpy as np

def detectAndDisplay(frame):
    font = cv.FONT_HERSHEY_SIMPLEX
      
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    gray = cv.medianBlur(gray, 5)
    
    rows = gray.shape[0]
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows/8,
                               param1=100, param2=30,
                               minRadius=5, maxRadius=50)

#    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) 
#    # color range
#    lower_red1 = np.array([0,100,100])
#    upper_red1 = np.array([10,255,255])
#    lower_red2 = np.array([160,100,100])
#    upper_red2 = np.array([180,255,255])
#    lower_green = np.array([40,50,50])
#    upper_green = np.array([90,255,255])
#    # lower_yellow = np.array([15,100,100])
#    # upper_yellow = np.array([35,255,255])
#    lower_yellow = np.array([15,150,150])
#    upper_yellow = np.array([35,255,255])
#    mask1 = cv.inRange(hsv, lower_red1, upper_red1)
#    mask2 = cv.inRange(hsv, lower_red2, upper_red2)
#    maskg = cv.inRange(hsv, lower_green, upper_green)
#    masky = cv.inRange(hsv, lower_yellow, upper_yellow)
#    maskr = cv.add(mask1, mask2)   
    
#    circles = cv.HoughCircles(maskr, cv.HOUGH_GRADIENT, 1, 80,
#                               param1=50, param2=10, minRadius=0, maxRadius=30)    
    
    
#    size = frame.shape
#    r = 5
#    bound = 4.0 / 10
    
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            print(center)
            # circle center
            cv.circle(frame, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv.circle(frame, center, radius, (255, 0, 255), 3)


#    if circles is not None:
#        circles = np.uint16(np.around(circles))

#        for i in circles[0, :]:
#            if i[0] > size[1] or i[1] > size[0]or i[1] > size[0]*bound:
#                continue

#            h, s = 0.0, 0.0
#            for m in range(-r, r):
#                for n in range(-r, r):

#                    if (i[1]+m) >= size[0] or (i[0]+n) >= size[1]:
#                        continue
#                    h += maskr[i[1]+m, i[0]+n]
#                    s += 1
#            if h / s > 50:
#                cv.circle(frame, (i[0], i[1]), i[2]+10, (0, 255, 0), 2)
#                cv.circle(maskr, (i[0], i[1]), i[2]+30, (255, 255, 255), 2)
#                cv.putText(frame,'RED',(i[0], i[1]), font, 1,(255,0,0),2,cv.LINE_AA)
#                cv.imshow("detected circles", frame)

    cv.imshow("detected circles", frame)



# camera_device = args.camera
#-- 2. Read the video stream
cap = cv.VideoCapture(0)
if not cap.isOpened:
    print('--(!)Error opening video capture')
    exit(0)
    
while True:
    ret, frame = cap.read()
    if frame is None:
        print('--(!) No captured frame -- Break!')
        break
    detectAndDisplay(frame)
    if cv.waitKey(10) == 27:
        break
