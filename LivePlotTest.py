
"""Grabs Data from USB Serial Connection then displays it on live graph."""

import matplotlib.pyplot as plt
from drawnow import *
import serial
from collections import deque

COMM_PORT = input("Comm Port: ")
BAUD_RATE = input("BaudRate: ")


#Sets to Default Values
if(COMM_PORT==""):
    COMM_PORT = "COM3"
if(BAUD_RATE==""):
    BAUD_RATE = 115200
else:
    BAUD_RATE = int(BAUD_RATE)

    


serialObj = serial.Serial(COMM_PORT, BAUD_RATE,timeout=None)

data_list = []

plt.ion() #matplotlob in interactive mode
cnt = 0

def makeFig():
   plt.plot(data_list)
   plt.ylim(-1.6, 1.6)
   plt.title('Live Stream Data')
   
   
   
while True:
    while (serialObj.inWaiting() ==0):
       pass
       
    try:
       serialObj.reset_input_buffer()
       raw_data = serialObj.readline()
       data = raw_data.decode("utf-8").strip()
       data_list.append(float(data))
       print(data)
       serialObj.reset_input_buffer() 
    except(KeyboardInterrupt):
       raise    
    except:
       pass
       
    
    drawnow(makeFig)
    plt.pause(0.0000001)
    cnt += 1
    if(cnt>50):
       data_list.pop(0)
       
    
     
       
       
       
       
       
       
       
       
       
       
       
       
       
       
       