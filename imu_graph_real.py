#graphing data in real time from imu data

import mpu6050v2
import time
import datetime as dt
import matplotlib.pyplot as plt
import numpy as np
import math
import matplotlib.animation as animation

#for dynamic plotting
fig = plt.figure()
ax = fig.add_subplot(1,1,1)
xs = []
ys = []
ys1 = []
ys2 = []

#start communication with MPU6050 (in module)
mpu6050v2.imu_init()

print (" ~MPU6050 Gyro + Accel~ ")

#i is the frame number
def animate(i,xs,ys,ys1,ys2):

    imu_values = mpu6050v2.read_imu() #array of 6 readings

    #xs.append(dt.datetime.now().strftime('%H:%M:%S'))
    xs.append(i)
    ys.append(imu_values[0])
    ys1.append(imu_values[1])
    ys2.append(imu_values[2])

    xs = xs[-20:]    
    ys = ys[-20:]
    ys1 = ys1[-20:]
    ys2 = ys2[-20:]

    i = i + 1
    
    #draw x and y lists everytime
    ax.clear()
    ax.plot(xs,ys,xs,ys1,xs,ys2)
    
    #formatting
    plt.title('IMU Real Time')
    plt.ylabel('Accel (X,Y,Z)')
    
ani = animation.FuncAnimation(fig,animate,fargs=(xs,ys,ys1,ys2),interval=500)
plt.show()
