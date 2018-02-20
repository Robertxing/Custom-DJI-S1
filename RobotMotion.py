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




