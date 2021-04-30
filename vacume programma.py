# -*- coding: utf-8 -*-
"""
Created on Mon Apr 12 12:20:14 2021

@author: Mees Wesseling
"""

import serial # you need to install the pySerial :pyserial.sourceforge.net
import time
# your Serial port should be different!
arduino = serial.Serial('/dev/tty.usbmodem1411', 9600)

def onOffFunction():
    vacume = arduino.readline()  #A if vacume is on B if off
    print(vacume)
    command = raw_input("Type something..: (on/ off / bye )");
    if command =="on":
        print("The LED is on...")
        time.sleep(1) 
        arduino.write('H') 
        onOffFunction()
    elif command =="off":
        print("The LED is off...")
        time.sleep(1) 
        arduino.write('L')
        onOffFunction()
    elif command =="bye":
        print("conection lost")
        time.sleep(1) 
        arduino.close()
    else:
        print("Erros")
        onOffFunction()
    

time.sleep(2) #waiting the initialization...

onOffFunction()