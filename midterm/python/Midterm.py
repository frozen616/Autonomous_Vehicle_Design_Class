# Plot 4 signals in real time. 
# programmer Tyler Angus 
# used to analyze data coming from the self balancing robot kit

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
    #initialize the graph data sets 
    graph_presAngle.set_data([], [])
    graph_error.set_data([], [])
    graph_pid.set_data([], [])
    graph_motorControl.set_data([], [])
 
    return graph_presAngle, graph_error, graph_pid, graph_motorControl

def animate(i):
    global t, presAngle, error, pid, motorControl
  

    while (ser.inWaiting() == 0):
        pass

    arduinoString = ser.readline().decode("utf-8") #decodes the information sent through serial
    dataArray = arduinoString.split(',') #seperates the info into an array and uses the ',' to know where to split the data 

    #This is where we take the data from the data array and append it to some globals
    presAngle.append(float(dataArray[0]))    
    error.append(float(dataArray[1]))    
    pid.append(float(dataArray[2]))
    motorControl.append(float(dataArray[3]))


    presAngle.pop(0)
    error.pop(0)
    pid.pop(0)
    motorControl.pop(0)



    graph_presAngle.set_data(t, presAngle)
    graph_error.set_data(t, error)
    graph_pid.set_data(t, pid)
    graph_motorControl.set_data(t, motorControl)



    return graph_presAngle, graph_error, graph_pid, graph_motorControl

   

if __name__ == '__main__':

    portName = "/dev/ttyUSB2"
    ser = serialConnect(portName,115200)
    sleep(2)                                        # give Arduino time to reset

    # flush input buffer, discarding all contents
    ser.reset_input_buffer()

    numPoints = 201                                 # number of data points

    
    fig = plt.figure(figsize=(7, 4))               # create figure window
    ax = plt.axes(xlim=(0,numPoints-1), ylim=(-45, 45))    # specify axis limits

    plt.title('Real-time data')
    plt.xlabel('Data points')
    plt.ylabel('Error')
    ax.grid(True)

    
    graph_error, = ax.plot([], [], 'b', label = 'Error from Desired Angle') #graph the error make a blue line and label it Error from desired Angle
    

    ax.legend(loc='upper right')



    #creating the figure for the plot of PID 
    fig1 = plt.figure(figsize=(7,4))
    ax = plt.axes(xlim=(0,numPoints-1), ylim=(-1000, 1000))    # specify axis limits

    #label the plot with info 
    plt.title('Real-time  data')
    plt.xlabel('Data points')
    plt.ylabel('PID')
    ax.grid(True)

    graph_pid, = ax.plot([], [], 'b', label = 'PID Output')
    
    #this says where to put the legend 
    ax.legend(loc='upper right')


    #creating a third figure to plot the motor control 
    fig2 = plt.figure(figsize=(7,4))
    ax = plt.axes(xlim=(0,numPoints-1), ylim=(-500, 500))

    plt.title('Real-time  data')
    plt.xlabel('Data points')
    plt.ylabel('Motor Speed')
    ax.grid(True)

    graph_motorControl, = ax.plot([],  'b', label = 'Motor Control')

    ax.legend(loc='upper right')



    fig3 = plt.figure(figsize=(7,4))
    ax = plt.axes(xlim=(0,numPoints-1), ylim=(60, 120))

    plt.title('Real-time  data')
    plt.xlabel('Data points')
    plt.ylabel('Present Angle')
    ax.grid(True)

    graph_presAngle, = ax.plot([], [], 'b', label = 'Present Angle of Bot')


    ax.legend(loc='upper right')




    t = list(range(0, numPoints))
    presAngle = []
    error = []
    pid = []
    motorControl = []



    for i in range(0, numPoints):
        presAngle.append(0)
        error.append(0)
        pid.append(0)
        motorControl.append(0)



    delay = 20
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                               interval=delay, blit=True)

    anim = animation.FuncAnimation(fig1, animate, init_func=init,
                               interval=delay, blit=True)

    anim = animation.FuncAnimation(fig2, animate, init_func=init,
                               interval=delay, blit=True)

    anim = animation.FuncAnimation(fig3, animate, init_func=init,
                               interval=delay, blit=True)
    plt.show()


