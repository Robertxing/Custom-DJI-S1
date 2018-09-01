#adapted from ElectricWings tutorial and
#http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html

import smbus            #import SMBus module of I2C
from time import sleep  #import

#MPU6050 Registers
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT = 0x3B
ACCEL_YOUT = 0x3D
ACCEL_ZOUT = 0x3F
GYRO_XOUT  = 0x43
GYRO_YOUT  = 0x45
GYRO_ZOUT  = 0x47

bus = smbus.SMBus(1)
imu1 = 0x68   # MPU6050 #1 address
#imu2 = 0x69   # MPU6050 #2 address (set AD0 to High)

def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(imu1, SMPLRT_DIV, 7)

    #Write to power management register
    bus.write_byte_data(imu1, PWR_MGMT_1, 1)

    #Write to Configuration register
    bus.write_byte_data(imu1, CONFIG, 0)

    #Write to Gyro configuration register
    bus.write_byte_data(imu1, GYRO_CONFIG, 24)

    #Write to interrupt enable register
    bus.write_byte_data(imu1, INT_ENABLE, 1)

def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(imu1, addr)
    low = bus.read_byte_data(imu1, addr+1)

    #concatenate higher and lower value
    value = (high << 8) | low

    #to get signed value from mpu6050
    if(value > 32768): #2^15
        value = value - 65536 #2^16
    return value

MPU_Init() #initialization of MPU6050

print (" ~MPU6050 Gyro+ Accel~ ")

while True:

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

    print ("Gx%.2f" %Gx,"Gy%.2f" %Gy,"Gz%.2f" %Gz)
    print ("Ax%.2f" %Ax,"Ay%.2f" %Ay,"Az%.2f" %Az)

    sleep(1)
