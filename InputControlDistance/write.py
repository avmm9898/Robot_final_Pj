import serial
from time import sleep
import sys

COM_PORT = 'COM10'  
BAUD_RATES = 9600
ser = serial.Serial(COM_PORT, BAUD_RATES)

try:
    while True:
        cmd = input('speed,distance,direction:\n').lower()
        ser.write((cmd + '\n').encode())  
        while ser.in_waiting:
            mcu_feedback = ser.readline().decode() 
            print('board response' + mcu_feedback)
            
except KeyboardInterrupt:
    ser.close()
    print('bye')
