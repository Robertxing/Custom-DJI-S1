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



