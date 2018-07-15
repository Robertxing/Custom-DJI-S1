#Open and Closed Loop Control (using IMU)

#Open loop control
'''
#using MotorClass

from gpiozero import Motor
from time import sleep

motor1 = Motor(4,14) #Right Motor
motor2 = Motor(17,27) #Left Motor

#robot goes forward at 0.25 speed
sleep(5)
motor1.forward(0.25) 
motor2.forward(0.25)

#To make robot veer to right. expected disturbance
sleep(5)
motor1.forward(0.3) 
motor2.forward(0.5)
'''

#Closed loop implementation. Using IMU

from gpiozero import Motor
from time import sleep
import cv2
import smbus
import math

power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

bus = smbus.SMBus(1)
address = 0x68 #read from i2cdetect -y 1 print out

#I2C Functions
def read_byte(adr):
    return bus.read_byte_data(address,adr)

def read_word(adr):
    high = bus.read_byte_data(address,adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

#For gyroscope
def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535-val)+1)
    else:
        return val

motor1 = Motor(4,14) #Right Motor
motor2 = Motor(17,27) #Left Motor

sleep(5) #wait 5 seconds before starting

while True:

    #read gyro
    gyro_zout = read_word_2c(0x47) #z-direction

    #basic gyroscope reading for feedback control
    rate_gyro_z = gyro_zout*0.07 #sensitivity level of 2000 dps
    
    #EWMA Filter for feedback control
        
    velocityR = 0.5 - 0.02*rate_gyro_z
    
    if velocityR > 1:
        velocityR = 1   
    elif velocityR < 0:
        velocityR = 0
        
    motor1.forward(velocityR)

    velocityL = 0.5 - 0.02*rate_gyro_z
    
    if velocityL > 1:
        velocityL = 1
    elif velocityL < 0:
        velocityL = 0
        
    motor2.forward(velocityL)

    sleep(2)
    
    key = cv2.waitKey(5) & 0xFF
    if key == ord("q"):
        break






