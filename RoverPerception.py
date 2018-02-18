#Ideas for robotic perception with PiCamera

'''
1. Region of interest
2. Edge detection
3. Tracking red ball (using differential wheel design)
4. Threshold and warping to find obstacle free path
5. ML for image classification (later)

'''
#Test Code

#Highlighting Red ball

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
