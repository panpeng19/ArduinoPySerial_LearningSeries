# -*- coding: utf-8 -*-
"""
Created on Wed Jun 19 16:28:44 2019
Updated on Wed May 13 14:15:33 2020
@author: Lionel, Peng
"""



import serial
import pickle

ser = serial.Serial('/dev/cu.usbmodem1421')   #if use serial programming port, 
                                              #should add argument:baudrate = 250000                                                                                  #should add argument:baudrate = 250000
ser.flushInput()


while True:
    print("Connected with:", ser.name)       #print the connected serial port name 
    try:
        ser_bytes = ser.read(16384)          #read up to 16384 bytes 
        with open("mypicklefile19", "wb") as f:
            pickle.dump(ser_bytes, f)
        
    except:
        print("Keyboard Interrupt")
        break