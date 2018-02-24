#Testing IMU
#(adapted from http://blog.bitify.co.uk/2013/11/reading-data-from-
#mpu-6050-on-raspberry.html)

import smbus
import math
import cv2
from time import sleep

power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

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

#For accelerometer
    
def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x,dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y,dist(x,z))
    return math.degrees(radians)

bus = smbus.SMBus(1)
address = 0x68 #read from i2cdetect -y 1 print out

#wakes up MPU-9250
#bus.write_byte_data(address,power_mgmt_1,0)
    
while True:

    print("gyro data")
    print("---------")

    #Test Gyroscope
    #taken from MPU9250 DataSheet
    gyro_xout = read_word_2c(0x43)
    gyro_yout = read_word_2c(0x45)
    gyro_zout = read_word_2c(0x47)
    
    gyro_xscaled = gyro_xout/131 #to get degrees/sec
    gyro_yscaled = gyro_yout/131 
    gyro_zscaled = gyro_zout/131 

    print ("gyro_x: ", gyro_xscaled)
    print ("gyro_y: ", gyro_yscaled)
    print ("gyro_z: ", gyro_zscaled)

    #Test Accelerometer
    accel_xout = read_word_2c(0x3b)
    accel_yout = read_word_2c(0x3d)
    accel_zout = read_word_2c(0x3f)

    accel_xscaled = accel_xout / 16384.0
    accel_yscaled = accel_yout / 16384.0
    accel_zscaled = accel_zout / 16384.0

    print ("scaled accel_xout: ", accel_xscaled)
    print ("scaled accel_yout: ", accel_yscaled)
    print ("scaled accel_zout: ", accel_zscaled)

    sleep(2)
