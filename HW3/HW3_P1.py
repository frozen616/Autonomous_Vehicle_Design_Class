'''
Lesson 5 Description: Extracts x, y values from serial data read

Uses CTRL+C signal handler to terminate while loop execution

'''
import serial
from signal import signal, SIGINT
from time import sleep

keepRunning = True 

def handler(signal, frame):
    global keepRunning 
    print('SIGINT or CTRL+C detected, setting keepRunning to False')
    keepRunning = False


def serialConnect(portName):
    try: 
        ser = serial.Serial(portName, 38400)
        print("opened port " + ser.name + '\n')
        # give Arduino time to reset
        sleep(2)
        # flush input buffer, discarding all contents
        ser.reset_input_buffer()
        return ser 
    except serial.SerialException:
        raise IOError("problem connecting to " + portName)




if __name__ == '__main__':

    #register the signal handler
    signal(SIGINT, handler)

    portName = "/dev/ttyUSB3"
    ser = serialConnect(portName)
    
    file = open("HW3_#1.txt", 'w')
    while keepRunning == True:

        if ser.in_waiting > 0:
            bytesRead = ser.read_until()
            read_list = [int(v) for v in bytesRead.decode().split(':')]
            if len(read_list) == 13: 
                
                ax_m = read_list[0]
                ay_m = read_list[1]
                az_m = read_list[2]
                gx_m = read_list[3]
                gy_m = read_list[4]
                gz_m = read_list[5]

                ax_o = read_list[6]
                ay_o = read_list[7]
                az_o = read_list[8]
                gx_o = read_list[9]
                gy_o = read_list[10]
                gz_o = read_list[11]

                loop_it = read_list[12]

                print("ax_mean: \t" + str(ax_m) + ", \tay_mean: \t" + str(ay_m) + ", \taz_mean: \t" + str(az_m))
                print("gx_mean: \t" + str(gx_m) + ", \tgy_mean: \t" + str(gy_m) + ", \tgz_mean: \t" + str(az_m))
                print("ax_offset: \t" + str(ax_o) + ", \tay_offset: \t" + str(ay_o) + ", \taz_offset: \t" + str(az_o))
                print("gx_offset: \t" + str(gx_o) + ", \tgy_offset: \t" + str(gy_o) + ", \tgz_offset: \t" + str(gz_o))
                print("loop iteration #" + str(loop_it))
                print("   ")

                file.write("ax_mean: \t" + str(ax_m) + ", \tay_mean: " + str(ay_m) + ", \t\taz_mean: " + str(az_m)+ '\n')
                file.write("gx_mean: \t" + str(gx_m) + ", \tgy_mean: " + str(gy_m) + ", \t\tgz_mean: " + str(az_m)+ '\n')
                file.write("ax_offset: \t" + str(ax_o) + ", \tay_offset: " + str(ay_o) + ", \taz_offset: " + str(az_o)+ '\n')
                file.write("gx_offset: \t" + str(gx_o) + ", \tgy_offset: " + str(gy_o) + ", \tgz_offset: " + str(gz_o)+ '\n')
                file.write("loop iteration #" + str(loop_it)+ '\n'+ '\n')
                file.write("   ")


            else:
                print("warning read_list length is " + len(read_list))
                
         
    
    

        # kill a bit of time before running through loop again
        # Arduino program transmitting every 300 ms
        sleep(30/1000)

    print('while loop terminated')
    file.close()  
    ser.close()
    print("closed port")



