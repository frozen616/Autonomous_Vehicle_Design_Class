from vpython import *
import numpy as np
from time import *
import math
import serial

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

scene.rang=5
toRad=2*np.pi/360
toDeg=1/toRad
scene.forward-vector(-1,-1,-1)

scene.width = 900
scene.height = 600

if __name__ == '__main__':

    portName = "/dev/ttyUSB0"
    ser = serialConnect(portName,115200)
    sleep(2)

    # flush input buffer, discarding all contents
    ser.reset_input_buffer()

    frame=box(length =4, width = 5, height=5, color=color.purple,opacity=.3)
    axel=cylinder(axis=vector(0,0,1),pos=vector(0,-2.6,-3),length =6,radius=.3,color=color.yellow,opacity=.3)
    wheel1=cylinder(axis=vector(0,0,1),pos=vector(0,-2.6,2.5),length = 1,radius=1, color=color.blue, opacity=.3)
    wheel2=cylinder(axis=vector(0,0,1),pos=vector(0,-2.6,-3.5),length=1, radius=1, color=color.blue, opacity=.3)
    Xarrow=arrow(axis=vector(1,0,0),length=16,shaftwidth=.1,color=color.red)
    Yarrow=arrow(axis=vector(0,1,0),length=16,shaftwidth=.1,color=color.green)
    Zarrow=arrow(axis=vector(0,0,1),length=16,shaftwidth=.1,color=color.blue)

    leftArrow=arrow(axis=vector(1,0,0),length=16,shaftwidth=.1,color=color.red)
    topArrow=arrow(axis=vector(0,1,0),length=16,shaftwidth=.1,color=color.green)
    frontArrow=arrow(axis=vector(0,0,1),length=16,shaftwidth=.1,color=color.blue)

    robotObj = compound([frame,axel,wheel1,wheel2])
    j=0

    while True:

        while (ser.inWaiting() == 0):
            pass

        arduinoString = ser.readline().decode("utf-8")
        dataArray = arduinoString.split(',')

        roll=float(dataArray[6])*toRad*-1
        pitch=float(dataArray[5])*toRad*-1
        yaw=float(dataArray[4])*toRad*-1

        

        rate(50)
        k=vector(cos(yaw)*cos(pitch), sin(pitch), sin(yaw)*cos(pitch))
        y=vector(0,1,0)
        s=cross(k,y)
        v=cross(s,k)
        vrot=v*cos(roll)+cross(k,v)*sin(roll)

        frontArrow.axis=s
        leftArrow.axis=k
        topArrow.axis=-vrot
        leftArrow.length=16
        frontArrow.length=16
        topArrow.length=-16

        robotObj.axis=k
        robotObj.up=vrot



    



