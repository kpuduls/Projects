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

imagePath = R'C:\SightLine Applications\SLA-Examples-ARM 3.02.04\Classifier\unprocessed images\fixedwingsamples\fixedwingmid'
dirName = R'C:\SightLine Applications\SLA-Examples-ARM 3.02.04\Classifier\unprocessed images\sampleimages'
bgd_dir = R'C:\SightLine Applications\SLA-Examples-ARM 3.02.04\Classifier\unprocessed images\backgroundsamples'


# Create target Directory if don't exist
if not os.path.exists(dirName):
    os.mkdir(dirName)
    print("Directory " , dirName ,  " Created ")
else:    
    print("Directory " , dirName ,  " already exists")

images = load_images_from_folder(imagePath)

# Create target Directory if don't exist
if not os.path.exists(bgd_dir):
    os.mkdir(bgd_dir)
    print("Directory " , bgd_dir ,  " Created ")
else:    
    print("Directory " , bgd_dir ,  " already exists")

images = load_images_from_folder(imagePath)

n = 0
print("n zeroed")

for image in images:
    x, y, sample = create_sample(image)
    filename = "filename_{0}.png".format(n)
    os.chdir(dirName)
    cv2.imwrite(filename, sample)
    print("\n saved not rotated file: {0}".format(filename))
    n = n + 1
    
    print("new image")
    for angle in np.arange(0,360,30):
        samplerot = rotated_sample(x, y, image, angle)
        filename = "filename_{0}.png".format(n)
        cv2.imwrite(filename, samplerot)
        n = n +1
        print("\n saved rotated file: {0}".format(filename))
    
h,w = image.shape[:-1]
imgSize = 64
    
os.chdir(bgd_dir)

length = len(images)

for image in images[:4]:    
    for col in np.arange(0, w, imgSize):
        for row in np.arange(0, h, imgSize):
            if abs((col + imgSize/2) - x) > imgSize/2:
                if abs((row + imgSize/2) - y) > imgSize/2:
                    bgd = image[row : row + imgSize, col : col + imgSize]
                    filename = "filename_{0}.png".format(n)
                    cv2.imwrite(filename, bgd)
                    n = n + 1
