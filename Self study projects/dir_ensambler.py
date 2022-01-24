# Python program to explain cv2.imwrite() method
  
# importing cv2 
import os
import cv2
import numpy as np
from image_processer import create_sample
from rotate_sample import rotated_sample

def load_images_from_folder(folder):
    images = []
    for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder,filename), cv2.IMREAD_COLOR)
        if img is not None:
            images.append(img)
    return images

imagePath1 = R'dir'
imagePath2 = R'dir'
imagePath3 = R'dir'
dirName = R'dir'


# Create target Directory if don't exist
if not os.path.exists(dirName):
    os.mkdir(dirName)
    print("Directory " , dirName ,  " Created ")
else:    
    print("Directory " , dirName ,  " already exists")

images1 = load_images_from_folder(imagePath1)
images2 = load_images_from_folder(imagePath2)
images3 = load_images_from_folder(imagePath3)

n = 0
print("n zeroed")

for image in images1:
    filename = "filename_{0}.png".format(n)
    os.chdir(dirName)
    cv2.imwrite(filename, image)
    print("\n saved not rotated file: {0}".format(filename))
    n = n + 1
    
for image in images2:
    filename = "filename_{0}.png".format(n)
    os.chdir(dirName)
    cv2.imwrite(filename, image)
    print("\n saved not rotated file: {0}".format(filename))
    n = n + 1

for image in images3:
    filename = "filename_{0}.png".format(n)
    os.chdir(dirName)
    cv2.imwrite(filename, image)
    print("\n saved not rotated file: {0}".format(filename))
    n = n + 1
