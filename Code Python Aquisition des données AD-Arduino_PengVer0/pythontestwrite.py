# -*- coding: utf-8 -*-
"""
Created on Mon May 20 15:13:17 2019

@author: Lionel
"""

import serial
import time
import csv

ser = serial.Serial('COM14')
ser.flushInput()

while True:
    try:
        ser_bytes = ser.read(16384)
        decoded_bytes =  ser_bytes
        print(decoded_bytes)
        with open("test_data.csv","a") as f:
            writer = csv.writer(f,delimiter=",")
            writer.writerow([time.perf_counter(),ser_bytes])
    except:
        print("Keyboard Interrupt")
        break