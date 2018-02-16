#introduction to openCV with Raspberry Pi

#From pyimagesearch.com

#testing camera photo

#import packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

#initialize camera
camera = PiCamera()
raw = PiRGBArray(camera)

time.sleep(0.1)

camera.capture(raw, format="bgr")
image = raw.array

cv2.imshow("ImageTest", image)
cv2.waitKey(0)

#testing camera video

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
 
# allow the camera to warmup
time.sleep(0.1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array
 
	# show the frame
	cv2.imshow("Frame", image)
	key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

#OpenCV Tutorials

#(1)Core Operations

import cv2
import numpy as np

img = cv2.imread('test1.JPG',1)
img = cv2.resize(img,(500,420))

#img[:,:,2] = 0 #remove R pixels. opencv has  BGR

size = img.shape
print size

img2 = img[100:400,100:300] #ROI

#using array.item() and array.itemset() with numpy is better

cv2.imshow('image',img2)
cv2.waitKey(5000)
cv2.destroyAllWindows()

#(2) Image Processing

#1. Changing Colorspaces

#extracting single color from video (red ball)
# 1. Take each frame
# 2. Convert to BGR to HSV (Hue Saturation Value)
# 3. Threshold image for range of red
# 4. Extract red object

import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera

camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 28
frameCapture = PiRGBArray(camera,size = (640,480))

for frame in camera.capture_continuous(frameCapture,format = "bgr", use_video_port=True):

    image = frame.array

    image = cv2.flip(image,0) #flip image to correct way up
    image = cv2.flip(image,1) #flip image left to right
    
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    blueLower = np.array([70,50,110])
    blueUpper = np.array([255,255,130])

   #For adaptive thresholding. using mean method
   result = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY) #make grayscale 
   cv2.adaptiveThreshold(result,cv2.ADAPTIVE_THRESH_MEAN_C, \ 
	cv2.THRESH_BINARY,11,4)
	
    mask = cv2.inRange(hsv, blueLower, blueUpper)
    result = cv2.bitwise_and(image,image, mask = mask)
    
    cv2.imshow("Result",result)
    key = cv2.waitKey(5) & 0xFF

    frameCapture.truncate(0)

    if key == ord("q"):
        break

#3. Geometric Transformations

#translation = shift object's position. using matrix [1,0,t_x;0,1,t_y]

import cv2
import numpy as np

image = cv2.imread('test2.JPG',0)
image = cv2.resize(image,(500,420))
rows,cols = image.shape

M = np.float32([[1,0,50],[0,1,50]])
dst = cv2.warpAffine(image,M,(cols,rows))

cv2.imshow('image',dst)
cv2.waitKey(5000)
cv2.destroyAllWindows()

#rotation = rotation of image by angle theta. matrix is [cos(theta),-sin(theta) ;
# sin(theta), cos(theta)]

import cv2
import numpy as np

image = cv2.imread('test3.JPG',0)
image = cv2.resize(image,(500,420))
rows,cols = image.shape

M = cv2.getRotationMatrix2D((cols/2,rows/2),90,1)
dst = cv2.warpAffine(image,M,(cols,rows))

cv2.imshow('image',dst)
cv2.waitKey(5000)
cv2.destroyAllWindows()

#affine transformation






