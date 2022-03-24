# Standard imports
import cv2
import numpy as np;
# importing os module  
import os

def create_sample(img):
    # Using cv2.imread() method
    # to read the image

    # get gray and rgb images
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    blur = cv2.GaussianBlur(gray, (15, 15),cv2.BORDER_DEFAULT)

    ret,threshold = cv2.threshold(gray,140,255,cv2.THRESH_BINARY_INV)

    contours, hierarchies = cv2.findContours(threshold, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    
    amax = 0
    
    for i in contours:
        M = cv2.moments(i)
        area = cv2.contourArea(i)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00']) 
            if (cy < 1000) and (cy > 10) and (cx < 1000) and (cx > 0):
                #cv2.drawContours(img, [i], -1, (0, 255, 0), 2)
                #cv2.circle(img, (cx, cy), 7, (0, 0, 255), -1)
                #cv2.putText(img, "center", (cx - 20, cy - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                #print("x: {0} y: {1} area: {2}".format(cx, cy, area))
                x = cx
                y = cy
                a = area
                if (a > amax) and (a < 100000):
                    amax = a
                    xmax = x
                    ymax = y
                    imax = i
                    print(xmax, ymax, amax)
                
    imgSize = 192
    #cv2.rectangle(img, (x-imgSize/2, y+imgSize/2), (x+imgSize/2, y-imgSize/2), (255, 255, 255), 1)
    
    #cv2.drawContours(img, [imax], -1, (0, 255, 0), 2)
    
    # Filename
    filename = 'coords.jpg'

    # Using cv2.imwrite() method
    # Saving the image
    #cv2.imwrite(filename, threshold)

    # Create a sample of the img
    sample = img[y-imgSize/2 : y+imgSize/2, x-imgSize/2 : x+imgSize/2 ]
    
    samplemax = img[ymax-imgSize/2 : ymax+imgSize/2, xmax-imgSize/2 : xmax+imgSize/2 ]
    
    
    return xmax, ymax, samplemax
    
    
    
    