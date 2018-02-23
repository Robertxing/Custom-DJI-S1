#Robot motion - testing the motor controller and Pi interface

#Basic On/off test for motors

from gpiozero import OutputDevice
from time import sleep

a = OutputDevice(4)
b = OutputDevice(14)

print ("Running")

for i in range(5):

    a.on()
    b.off()
    sleep(5)
    a.off()
    b.on()

#PWM test

from gpiozero import PWMOutputDevice as pwmd

a = pwmd(4)
b = pwmd(14)

a.value = 0.5
b.value = 0

#using MotorClass

from gpiozero import Motor

motor1 = Motor(4,14)
motor2 = Motor(17,27)

motor1.forward(0.25)
motor2.forward(0.25)

#using RobotClass

from gpiozero import Robot as Rover

rover = Rover(right = (4.14),left = (17,27))

rover.forward(100,3.0) #speed (1-255) and time in seconds
rover.backward(100,3.0)

#Basic Motion Testing:

#Box using RobotClass

from gpiozero import Robot
from time import sleep

rover = Robot((4,14),(17,27))

for i in range(4):
    rover.forward(0.2)
    sleep(5)
    rover.right(0.2)
    sleep(1)
rover.stop()

#Keyboard Controlled Robot (note: works in terminal not IDLE)

import curses as button
from gpiozero import Robot
import cv2

rover = Robot((4,14),(17,27))

#Create dictionary for button mapping
actions = {
    button.KEY_UP: rover.forward,
    button.KEY_DOWN: rover.backward,
    button.KEY_LEFT: rover.left,
    button.KEY_RIGHT: rover.right,
    }

def main(window):
    next_key = None
    while True:
        button.halfdelay(1)
        if next_key is None:
            key = window.getch()
        else:
            key = next_key
            next_key = None
        if key != -1:
            #Key is down
            button.halfdelay(3)
            action = actions.get(key)
            if action is not None:
                action()
            next_key = key
            while next_key == key:
                next_key = window.getch()
                #Key is up
            rover.stop()

        key = cv2.waitKey(5) & 0xFF

        if key == ord("q"):
            break

button.wrapper(main)

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
        return -((5535-val)+1)
    else:
        return val

motor1 = Motor(4,14) #Right Motor
motor2 = Motor(17,27) #Left Motor

sleep(5) #wait 5 seconds before starting

while True:

    #read gyro
    gyro_zout = read_word_2c(0x47) #z-direction

    #5 frame average filter
      
    G = gyro_zout/131
    #G = (G1 + G2 + G3 + G4 + G5)/5 #average

    #Proportional controller
    velocityR = 0.25 - (G/35)
    motor1.forward(velocityR)

    velocityL = 0.25 + (G/35)
    motor2.forward(velocityL)

    sleep(2)
    
    key = cv2.waitKey(5) & 0xFF
    if key == ord("q"):
        break






