# -*- coding: utf-8 -*-
"""
Created on Wed Jun 19 16:28:44 2019

@author: Lionel
"""



import serial
import pickle

ser = serial.Serial('COM14')
ser.flushInput()


while True:
    try:
        ser_bytes = ser.read(16384)
        with open("mypicklefile", "ab") as fichier:
            pickle.dump(ser_bytes, fichier)
        
    except:
        print("Keyboard Interrupt")
        break