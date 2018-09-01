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

#start communication with MPU6050 (in module)
mpu6050v2.imu_init()

print (" ~MPU6050 Gyro + Accel~ ")

#i is the frame number
def animate(i,xs,ys):

    acc_valueZ = mpu6050v2.read_imu()

    #xs.append(dt.datetime.now().strftime('%H:%M:%S'))
    xs.append(i)
    ys.append(acc_valueZ)

    xs = xs[-20:]    
    ys = ys[-20:]

    i = i + 1
    
    #draw x and y lists everytime
    ax.clear()
    ax.plot(xs,ys)
    
    #formatting
    plt.title('IMU Real Time AccZ')
    plt.ylabel('Accel_z (g)')
    
ani = animation.FuncAnimation(fig,animate,fargs=(xs,ys),interval=1000)
plt.show()







