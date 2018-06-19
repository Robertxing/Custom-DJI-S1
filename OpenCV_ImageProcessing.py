#(2) Image Processing with OpenCV

#2. Geometric Transformations

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

import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera

img = cv2.imread('test3.JPG')
img = cv2.resize(img,(500,420))
rows,cols,ch = img.shape

#affine transformation. All parallel lines in image will be
#parallel in the output image.

pts1 = np.float32([[50,50],[200,50],[50,200]])
pts2 = np.float32([[10,100],[200,50],[100,250]])

M = cv2.getAffineTransform(pts1,pts2)
dst = cv2.warpAffine(img,M,(cols,rows))

#perspective transformation
#3x3 transformation matrix. 4 points on input image with 3 not collinear

pts1 = np.float32([[50,50],[360,52],[28,364],[390,388]])
pts2 = np.float32([[0,0],[310,0],[0,310],[310,310]])

M = cv2.getPerspectiveTransform(pts1,pts2)
dst = cv2.warpPerspective(img,M,(310,310))

cv2.imshow("result", dst)
cv2.waitKey(5000)
cv2.destroyAllWindows()

#3. Smoothing Images

#2D Convolution
#low pass filters remove noise or blur image. Kernel - window centred on pixel.
#average of pixel values inside window. 

import cv2
import numpy as np

img = cv2.imread('test2.JPG')
img = cv2.resize(img,(500,420))

kernel = np.ones((3,3),np.float32)/9
dst = cv2.filter2D(img,-1,kernel)

cv2.imshow("Test Filter", img)
cv2.waitKey(5000)
cv2.destroyAllWindows()

#4. Morphological Transformations

#operations based on image shape. two inputs: Original image + kernel

#erosion: removes boundaries of foreground object. PIxel in original image will be
#a 1 if all pixels in kernel are 1 else made 0

import cv2
import numpy as np

img = cv2.imread('test2.JPG',0)
img = cv2.resize(img,(500,420))
kernel = np.ones((5,5),np.uint8)
erosion = cv2.erode(img,kernel,iterations=1)

cv2.imshow("Erosion Test",erosion)
cv2.waitKey(5000)
cv2.destroyAllWindows()

#dilation: pixel element is 1 if at least one pixel under kernel is 1.
#increases white region in image

import cv2
import numpy as np

img = cv2.imread('test2.JPG',0)
img = cv2.resize(img,(500,420))
kernel = np.ones((5,5),np.uint8)
dilation = cv2.dilate(img, kernel, iterations = 1)

cv2.imshow("DilationTest",dilation)
cv2.waitKey(5000)
cv2.destroyAllWindows()

