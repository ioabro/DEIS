# Ioannis Broumas
# ioabro17@student.hh.se
# Nov 2020
# Detect circles

import sys
import cv2 as cv
import numpy as np

def main():
    
    default_file = 'C:/Users/Giovani/Downloads/capture.png'
    # filename = argv[0] if len(argv) > 0 else default_file
    filename = default_file
    # Loads an image
    # src = cv.imread(cv.samples.findFile(filename), cv.IMREAD_COLOR)
    src = cv.imread(default_file, cv.IMREAD_COLOR)
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        #print ('Usage: hough_circle.py [image_name -- default ' + default_file + '] \n')
        return -1
    
    # Convert it to grayscale
    gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    # Apply a Median blur to reduce noise and avoid false circle detection
    gray = cv.medianBlur(gray, 5)
    
    rows = gray.shape[0]
    # Apply Hough Circle Transform
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 8,
                               param1=100, param2=30,
                               minRadius=5, maxRadius=50)
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # circle center
            center = (i[0], i[1])
            # print(center)
            cv.circle(src, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv.circle(src, center, radius, (255, 0, 255), 3)
    
    cv.imshow("detected circles", src)
    cv.waitKey(0)
    
    return 0

if __name__ == "__main__":
    main()