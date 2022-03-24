# Standard imports
import cv2
import numpy as np
import pandas as pd
# importing os module  
import os

# Import function to create the target sample
from sampler import create_sample

# Define video dir
video_dir = r'Z:\Personal Folder\Krisjanis Artis Puduls\Engine vibration test\old-engine-test\oldenginesdfiles\video6000straight_03_17_2022__16_14_45__0008.ts'
frame_dir = r'Z:\Personal Folder\Krisjanis Artis Puduls\Engine vibration test\Test Results\Old engine\video processing\6000'

# Go to dir for frame saving
os.chdir(frame_dir)

# Opens the Video file
cap= cv2.VideoCapture(video_dir)

data = {'x':[], 'y':[]}
df = pd.DataFrame(data)

print(df)

i=0
while(cap.isOpened()):
    ret, frame = cap.read()
    if ret == False:
        break
    # Do processing with frame
    x, y, sample = create_sample(frame)
    
    data2 = {'x': [x], 'y': [y]}
    print(data2)
        
    df2 = pd.DataFrame(data2)
    
    df = df.append(df2)
    
    print(df)
    
    #cv2.imwrite('old'+str(i)+'.jpg',sample)
    i+=1
    
df.to_csv(frame_dir + r"new.csv")

cap.release()
cv2.destroyAllWindows()