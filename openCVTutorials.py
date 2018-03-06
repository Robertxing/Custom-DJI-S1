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

