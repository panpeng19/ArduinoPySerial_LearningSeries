# -*- coding: utf-8 -*-
"""
Created on Wed Jun 19 16:28:44 2019

@author: Lionel
"""



import serial
import pickle

ser = serial.Serial('/dev/cu.usbmodem1421')   #, baudrate = 115200)
ser.flushInput()
fichier=open("mypicklefile-fake3", "wb")

print("Connected with:", ser.name)

while True:
  #  print("Connected with:", ser.name)
    try:
        ser_bytes = ser.read(16384)     #16384            #16384
        pickle.dump(ser_bytes, fichier)
        
    except:
        #time.sleep(10)
        #return ficher
        print("Keyboard Interrupt")
        break
