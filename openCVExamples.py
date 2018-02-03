#introduction to openCV with Raspberry Pi

#testing camera photo

#import packages
from picamera.array import piRGBArray
from picamera import PiCamera
import time
import cv2

#initialize camera
camera = PiCamera()
raw = piRGBArray(camera)

time.sleep(0.1)

camera.capture(raw, format="bgr")
image = raw.array

cv2.imshow("ImageTest", image)
cv2.waitKey(0)

#testing camera video



#(1)Core Operations - Basic operations on images

#modifying pixel values



