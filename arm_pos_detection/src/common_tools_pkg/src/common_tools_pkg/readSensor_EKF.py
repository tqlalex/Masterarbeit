#!/usr/bin/env python

import struct
import time
from threading import Thread

import numpy as np
import serial


class SerialRead:
    def __init__(self, serialPort, serialBaud, numParams):
        self.dataNumBytes = 4  # number of bytes of 1 data point
        self.numParams = numParams  # number of plots in 1 graph
        #self.rawData = bytearray(9 * self.dataNumBytes)
        self.rawData = bytearray(numParams * self.dataNumBytes)
        self.isRun = True
        self.isReceiving = False
        self.thread = None

        print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
            exit()

    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)

    def getSerialData(self, offset=np.zeros(9)):
        offset = np.zeros(self.numParams)
        privateData = self.rawData[:]
        #data = np.zeros(9)
        data = np.zeros(self.numParams)

        for x in range(self.numParams):
            #print(x)
            data[x], = struct.unpack('f', privateData[(x * self.dataNumBytes):(self.dataNumBytes + x * self.dataNumBytes)])
            #print(data[x])
        
        for j in range(int(self.numParams/9)):
            
            
            for i in range(9):

                # Gyr
                if i < 3:
                    data[i + 9 * j] = (data[i + 9 * j] - offset[i + 9 * j]) / 180.0 * np.pi  # convert from deg/s to rad/s
                    #print(j)
                    #print(i)
                    
                    #print(data[i+9*j])

                # Mag - todo: https://www.thepoorengineer.com/en/calibrating-the-magnetometer/
                elif i < 6:
                    data[i + 9 * j] -= offset[i + 9 * j]
                    #print(j)
                    #print(i)
                    
                    #print(data[i+9*j])
                # Acc
                elif i < 9:
                    data[i + 9 * j] -= offset[i + 9 * j]
                    #print(j)
                    #print(i)
                    
                    #print(data[i+9*j])
        return data

    def backgroundThread(self):  # retrieve data
        time.sleep(1.0)  # give some buffer time for retrieving data
        self.serialConnection.reset_input_buffer()
        while (self.isRun):
            self.serialConnection.readinto(self.rawData)
            self.isReceiving = True

    def close(self):
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected...')
