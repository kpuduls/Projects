import os
import cv2
import numpy as np

def rotated_sample(x, y, image, angle):
    image_center = (x,y)
    (h, w) = image.shape[:2]
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    rotated = cv2.warpAffine(image, rot_mat, (w,h))
    imgSize = 64
    result = rotated[y-imgSize/2 : y+imgSize/2, x-imgSize/2 : x+imgSize/2]
    print("angle in def {0}".format(angle))
    return result

def translated_sample(x,y, image, dx, dy):
    result = image[y-imgSize/2 + dy: y+imgSize/2 + dy, x-imgSize/2 + dx : x+imgSize/2 + dx]
    return result