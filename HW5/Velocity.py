# Plot 3 signals

import serial
import numpy as np
from time import sleep 
import csv
import matplotlib
import matplotlib.pyplot as plt 



ser = serial.Serial('/dev/ttyUSB9', 115200)
ser.flushInput()
file = open("HW_5_data.txt", 'w')
count = 0
i = 0
l_f_twenty_five = 0.0 # left motor foward at 25% duty cycle average variable
l_f_fifty = 0.0 
l_f_seventy_five = 0.0
l_b_twenty_five = 0.0
l_b_fifty = 0.0
l_b_seventy_five = 0.0
r_f_twenty_five = 0.0 #right motor foward at 25% duty cycle average variable
r_f_fifty = 0.0
r_f_seventy_five = 0.0
r_b_twenty_five = 0.0
r_b_fifty = 0.0
r_b_seventy_five = 0.0 #right motor backword at 75% dudty cycle average variable
bat_volt = 0.0

while count < 31: 
    
    if ser.in_waiting > 0:
        ser_bytes = ser.readline().decode("utf-8")
        decoded_bytes = (ser_bytes.split(','))
        decoded_bytes[5]='0'

        #disregard the first line of data because it is not accurate time
        if(count >= 1): 
            print(decoded_bytes)
            #write to the file the data collected through serial
            file.write('\n')
            file.write("count = ")
            file.write(str(decoded_bytes[0]))
            file.write("\tDt [ms] = ")
            file.write(str(decoded_bytes[1]))
            file.write("\tEncoder left = ")
            file.write(str(decoded_bytes[2]))
            file.write("\tEncoder right = ")
            file.write(str(decoded_bytes[3]))
            file.write("\tBattery Voltage = ")
            file.write(str(decoded_bytes[4]))
            bat_volt += float(decoded_bytes[4]) # the battery voltage doesn't change significantly with speed so I will use the same average for every test.
            #use if statements to add up the data as it comes in. 
            if count<=10 and i==0: #take the first ten tests and add them up 
                l_f_twenty_five += float(decoded_bytes[2])                
            if 10<count<=20 and i==0:
                l_f_fifty += float(decoded_bytes[2])
            if 20<count<=30 and i==0:
                l_f_seventy_five += float(decoded_bytes[2])
            if count<=10 and i==1:
                l_b_twenty_five += float(decoded_bytes[2])
            if 10<count<=20 and i==1:
                l_b_fifty += float(decoded_bytes[2])
            if 20<count<=30 and i==1:
                l_b_seventy_five += float(decoded_bytes[2])


            if count<=10 and i==0:
                r_f_twenty_five += float(decoded_bytes[3])
            if 10<count<=20 and i==0:
                r_f_fifty += float(decoded_bytes[3])
            if 20<count<=30 and i==0:
                r_f_seventy_five += float(decoded_bytes[3])
            if count<=10 and i==1:
                r_b_twenty_five += float(decoded_bytes[3])
            if 10<count<=20 and i==1:
                r_b_fifty += float(decoded_bytes[3])
            if 20<count<=30 and i==1:
                r_b_seventy_five += float(decoded_bytes[3])
        #This starts the loop over to take in the backward data i is used to seperate backward and forward data collection
        count = count + 1
        if (count == 31) and (i == 0):
            count = 0
            i = 1
            print("Backward Test: ")
            file.write('\n')
            file.write('Backword Data: ')
            file.write("\n")
#write to the file all of the sums of data
file.write('\n')
file.write('25% Left Motor Forward Total = ')
file.write(str(l_f_twenty_five))
file.write('\t 50% Left Motor Forward Total = ')
file.write(str(l_f_fifty))
file.write('  \t 75% Left Motor Forward Total = ')
file.write(str(l_f_seventy_five))
file.write('\n')

file.write('25% Left Motor Backward Total = ')
file.write(str(l_b_twenty_five))
file.write('\t 50% Left Motor Backward Total = ')
file.write(str(l_b_fifty))
file.write('\t 75% Left Motor Backward Total = ')
file.write(str(l_b_seventy_five))
file.write('\n')

file.write('25% Right Motor Forward Total = ')
file.write(str(r_f_twenty_five))
file.write('\t 50% Right Motor Forward Total = ')
file.write(str(r_f_fifty))
file.write('\t 75% Right Motor Forward Total = ')
file.write(str(r_f_seventy_five))
file.write('\n')

file.write('25% Right Motor Backward Total = ')
file.write(str(r_b_twenty_five))
file.write('\t 50% Right Motor Backward Total = ')
file.write(str(r_b_fifty))
file.write('\t 75% Right Motor Backward Total = ')
file.write(str(r_b_seventy_five))
file.write('\n')

#Average all of the data and store it 
l_f_twenty_five = l_f_twenty_five/10.0
l_f_fifty = l_f_fifty/10.0
l_f_seventy_five = l_f_seventy_five/10.0
l_b_twenty_five = l_b_twenty_five/10.0
l_b_fifty = l_b_fifty/10.0
l_b_seventy_five = l_b_seventy_five/10.0

r_f_twenty_five = r_f_twenty_five/10.0
r_f_fifty = r_f_fifty/10.0
r_f_seventy_five = r_f_seventy_five/10.0
r_b_twenty_five = r_b_twenty_five/10.0
r_b_fifty = r_b_fifty/10.0
r_b_seventy_five = r_b_seventy_five/10.0
#average the voltage 
bat_volt = bat_volt / 60.0

#write all of the averages into the file
file.write('\n')
file.write('Batter Voltage Average for tests = ')
file.write(str(bat_volt))
file.write('\n')

file.write('\n')
file.write('25% Left Motor Forward Average = ')
file.write(str(l_f_twenty_five))
file.write('\t 50% Left Motor Forward Average = ')
file.write(str(l_f_fifty))
file.write('\t 75% Left Motor Forward Average = ')
file.write(str(l_f_seventy_five))
file.write('\n')

file.write('25% Left Motor Backward Average = ')
file.write(str(l_b_twenty_five))
file.write('\t 50% Left Motor Backward Average = ')
file.write(str(l_b_fifty))
file.write('\t 75% Left Motor Backward Average = ')
file.write(str(l_b_seventy_five))
file.write('\n')

file.write('25% Right Motor Forward Average = ')
file.write(str(r_f_twenty_five))
file.write('\t 50% Right Motor Forward Average = ')
file.write(str(r_f_fifty))
file.write('\t 75% Right Motor Forward Average = ')
file.write(str(r_f_seventy_five))
file.write('\n')

file.write('25% Right Motor Backward Average = ')
file.write(str(r_b_twenty_five))
file.write('\t 50% Right Motor Backward Average = ')
file.write(str(r_b_fifty))
file.write('\t 75% Right Motor Backward Average = ')
file.write(str(r_b_seventy_five))
file.write('\n')


file.close()

#create a figure using bar graphs to represent the data
print("closed file")
ser.close()
print('closed Port')
        
labels = ['F 25%', 'F 50%', 'F 75%', 'B 25%', 'B 50%' ,'B 75%']
left_means = [l_f_twenty_five,l_f_fifty,l_f_seventy_five,l_b_twenty_five,l_b_fifty,l_b_seventy_five]
right_means= [r_f_twenty_five,r_f_fifty,r_f_seventy_five,r_b_twenty_five,r_b_fifty,r_b_seventy_five]

x = np.arange(len(labels)) #label locations
width = 0.45 # the width of the bars

fig, ax = plt.subplots()
rects1 = ax.bar(x - width/2, left_means, width, label = 'left motor')
rects2 = ax.bar(x + width/2, right_means, width, label = 'right motor') 

ax.set_ylabel('Encoder Rotations')
ax.set_title('Encoder data')
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.legend()

def autolabel(rects):
    #put labels above each bar
    for rect in rects:
        height = rect.get_height()
        ax.annotate('{}'.format(height),
                    xy=(rect.get_x()+ rect.get_width() / 2, height),
                    xytext=(0, 3), # 3 points vertical offset
                    textcoords="offset points",
                    ha='center', va='bottom')

autolabel(rects1)
autolabel(rects2)

fig.tight_layout()

plt.show()


