# Plot 3 signals

import serial
import numpy as np
from time import sleep 
from matplotlib import pyplot as plt
from matplotlib import animation


def serialConnect(portName, baudRate):
    try: 
        ser = serial.Serial(portName, baudRate)
        print("opened port " + ser.name + '\n')
        # give Arduino time to reset
        sleep(2)
        # flush input buffer, discarding all contents
        ser.reset_input_buffer()
        return ser 
    except serial.SerialException:
        raise IOError("problem connecting to " + portName)


def init():
    graph_aroll.set_data([], [])
    graph_groll.set_data([], [])
    graph_froll.set_data([], [])
    graph_apitch.set_data([], [])
    graph_gpitch.set_data([], [])
    graph_fpitch.set_data([], [])
    graph_gyaw.set_data([], [])   
    return graph_aroll, graph_groll, graph_froll, graph_apitch, graph_gpitch, graph_fpitch, graph_gyaw

def animate(i):
    global t, accelRoll, gyroRoll, cfilterRoll, accelPitch, gyroPitch, cfilterPitch, gyroYaw
  

    while (ser.inWaiting() == 0):
        pass

    arduinoString = ser.readline().decode("utf-8")
    dataArray = arduinoString.split(',')

    accelRoll.append(float(dataArray[0]))    
    gyroRoll.append(float(dataArray[3]))    
    CfilterRoll.append(float(dataArray[6]))
    accelPitch.append(float(dataArray[1]))
    gyroPitch.append(float(dataArray[2]))
    cfilterPitch.append(float(dataArray[5]))
    gyroYaw.append(float(dataArray[4]))

    accelRoll.pop(0)
    gyroRoll.pop(0)
    CfilterRoll.pop(0)
    accelPitch.pop(0)
    gyroPitch.pop(0)
    cfilterPitch.pop(0)
    gyroYaw.pop(0)


    graph_aroll.set_data(t, accelRoll)
    graph_groll.set_data(t, gyroRoll)
    graph_froll.set_data(t, CfilterRoll)
    graph_apitch.set_data(t, accelPitch)
    graph_gpitch.set_data(t, gyroPitch)
    graph_fpitch.set_data(t, cfilterPitch)
    graph_gyaw.set_data(t, gyroYaw)


    return graph_aroll, graph_groll, graph_froll, graph_apitch, graph_gpitch, graph_fpitch, graph_gyaw

   

if __name__ == '__main__':

    portName = "/dev/ttyUSB0"
    ser = serialConnect(portName,115200)
    sleep(2)                                        # give Arduino time to reset

    # flush input buffer, discarding all contents
    ser.reset_input_buffer()

    numPoints = 201                                 # number of data points
    fig = plt.figure(figsize=(7, 4))               # create figure window
    ax = plt.axes(xlim=(0,numPoints-1), ylim=(-180, 180))    # specify axis limits

    plt.title('Real-time sensor data')
    plt.xlabel('Data points')
    plt.ylabel('Rotation [Degrees]')
    ax.grid(True)

    graph_aroll, = ax.plot([], [], 'b', label = 'Accel Roll')
    graph_groll, = ax.plot([], [], 'r', label = 'Gyro Integration Roll')
    graph_froll, = ax.plot([], [], 'g', label = 'Filtered Roll')
    ax.legend(loc='upper right')
    ax.legend(loc='upper right')
    ax.legend(loc='upper right')

    fig1 = plt.figure(figsize=(7,4))
    ax = plt.axes(xlim=(0,numPoints-1), ylim=(-180, 180))    # specify axis limits

    plt.title('Real-time sensor data')
    plt.xlabel('Data points')
    plt.ylabel('Rotation [Degrees]')
    ax.grid(True)

    graph_apitch, = ax.plot([], [], 'b', label = 'Accel Pitch')
    graph_gpitch, = ax.plot([], [], 'r', label = 'Gyro Integration Pitch')
    graph_fpitch, = ax.plot([], [], 'g', label = 'Filtered Pitch')
    ax.legend(loc='upper right')
    ax.legend(loc='upper right')
    ax.legend(loc='upper right')

    fig2 = plt.figure(figsize=(7,4))
    ax = plt.axes(xlim=(0,numPoints-1), ylim=(-180, 180))    # specify axis limits

    plt.title('Real-time sensor data')
    plt.xlabel('Data points')
    plt.ylabel('Rotation [Degrees]')
    ax.grid(True)

    graph_gyaw, = ax.plot([], [], 'b', label = 'Gyro Yaw')

    ax.legend(loc='upper right')


    t = list(range(0, numPoints))
    accelRoll = []
    gyroRoll = []
    CfilterRoll = []
    accelPitch = []
    gyroPitch = []
    cfilterPitch = []
    gyroYaw = []


    for i in range(0, numPoints):
        accelRoll.append(0)
        gyroRoll.append(0)
        CfilterRoll.append(0)
        accelPitch.append(0)
        gyroPitch.append(0)
        cfilterPitch.append(0)
        gyroYaw.append(0)



    delay = 20
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                               interval=delay, blit=True)

    anim = animation.FuncAnimation(fig1, animate, init_func=init,
                               interval=delay, blit=True)

    anim = animation.FuncAnimation(fig2, animate, init_func=init,
                               interval=delay, blit=True)
    plt.show() 

