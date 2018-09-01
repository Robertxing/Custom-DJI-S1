#module for i2c communication (Pi and MPU6050)

import smbus            #import SMBus module of I2C
from time import sleep  #import

imu1 = 0x68   # MPU6050 #1 address
#imu2 = 0x69   # MPU6050 #2 address (set AD0 to High)
bus = None

#MPU6050 Registers
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38

ACCEL_XOUT = 0x3B
ACCEL_YOUT = 0x3D
ACCEL_ZOUT = 0x3F
GYRO_XOUT  = 0x43
GYRO_YOUT  = 0x45
GYRO_ZOUT  = 0x47

#initialize IMU
def imu_init():

    global bus

    bus = smbus.SMBus(1)
    
    #power management
    bus.write_byte_data(imu1, PWR_MGMT_1, 1)

    #Write to Configuration register
    bus.write_byte_data(imu1, CONFIG, 0)

    #write to sample rate register for gyro
    bus.write_byte_data(imu1, SMPLRT_DIV, 7)

    #gyro range, 3 = +/- 2000 degree/s
    bus.write_byte_data(imu1, GYRO_CONFIG,24) #0bxxx11xxx

    #accel range, 2 = +/- 8g
    bus.write_byte_data(imu1, ACCEL_CONFIG,16) #0bxxx10xxx

    #Write to Configuration register
    bus.write_byte_data(imu1, CONFIG, 0)

    #Write to interrupt enable register
    bus.write_byte_data(imu1, INT_ENABLE, 1)

def read_raw_data(addr):

    global bus

    #Accel and Gyro value are 16-bit
    high = bus.read_byte_data(imu1, addr)
    low = bus.read_byte_data(imu1, addr+1)

    #rearrange hi and lo bytes
    value = (high << 8) | low

    #to get signed value from mpu6050
    #if negative value. 2's complement
    if (value >= 0x8000): #2^16 = 65536
        return -((65535-value)+1)
    else:
        return value

def read_imu():

    global bus
    
    #Read Accelerometer raw value
    acc_x = read_raw_data(ACCEL_XOUT)
    acc_y = read_raw_data(ACCEL_YOUT)
    acc_z = read_raw_data(ACCEL_ZOUT)

    #Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT)
    gyro_y = read_raw_data(GYRO_YOUT)
    gyro_z = read_raw_data(GYRO_ZOUT)

    #Full scale range +/- 250 degree/C as per sensitivity scale factor
    Ax = acc_x/16384.0
    Ay = acc_y/16384.0
    Az = acc_z/16384.0

    Gx = gyro_x/131.0
    Gy = gyro_y/131.0
    Gz = gyro_z/131.0
    
    #append each value to array as 2-byte integers (word)    
    readings = [Ax,Ay,Az,Gx,Gy,Gz]

    return readings
