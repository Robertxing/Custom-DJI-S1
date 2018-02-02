#introduction to openCV with Raspberry Pi

#import packages
from picamera.array import piRGB
from picamera import PiCamera
import time
import cv2

#initialize camera
camera = PiCamera()
raw = piRGB(camera)

time.sleep(0.1)

camera.capture(raw, format="bgr")
image = raw.array

cv2.imshow("ImageTest", image)
cv2.waitKey(0)
