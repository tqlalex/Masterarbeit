#!/usr/bin/env python3
 
'''
File name: bluetooth_test.python
 
This program tests the Bluetooth connection between 
your PC and your robot.
The PC receives messages from the robot 
via Bluetooth and prints
those messages to your screen.
 
Modified from 
https://people.csail.mit.edu/albert/bluez-intro/x232.html
 
Author: Addison Sears-Collins
'''
 
import bluetooth # Import the python-bluez library
import rospy
from std_msgs.msg import String

import time
import struct
import numpy as np
##################################################
# Bluetooth parameters
 
robot_bluetooth_mac_address = '00:21:13:00:4A:96'
port = 1
pc_bluetooth_handle = None
data_size = 1024
 
######################################################
# Connect the PC's Bluetooth to the robot's Bluetooth
 
def connect():
  global pc_bluetooth_handle    
   
  while(True):    
    try:
      pc_bluetooth_handle = bluetooth.BluetoothSocket(
               bluetooth.RFCOMM)
      pc_bluetooth_handle.connect((
              robot_bluetooth_mac_address, port))
      break
    except bluetooth.btcommon.BluetoothError as error:
      pc_bluetooth_handle.close()
      print (
         "Could not connect: ", error, "; Retrying in 10s...")
      time.sleep(10)
   
  return pc_bluetooth_handle
 
# Connect to the robot's Bluetooth  
pc_bluetooth_handle = connect() 
  
#############################################################
# Main code
 
# If this file is the main (driver) program you are executing
if __name__ == '__main__': 
 
  #dataNumBytes = 4
  #numParams = 18
  #rawData = bytearray(numParams * dataNumBytes)
 
    pub = rospy.Publisher('Str_from_BT', String, queue_size = 10)
    rospy.init_node('BTReader', anonymous = True)

    try:
        byteStr = b'' 

        while(b'Header' not in byteStr):
        # Keep reading data from the robot
            incoming_data_from_robot = pc_bluetooth_handle.recv(
                        data_size)
  

            byteStr += incoming_data_from_robot
      
        byteStrList = byteStr.split(b'Header')


        while(True):

            byteStr = byteStrList[1]

            while(b'Header' not in byteStr):
            # Keep reading data from the robot
                incoming_data_from_robot = pc_bluetooth_handle.recv(
                            data_size)
  

                byteStr += incoming_data_from_robot
      
            byteStrList = byteStr.split(b'Header')

            bytesToSend = byteStrList[0]

            #print(bytesToSend)
            #print("length is: ", len(bytesToSend))


      
      
      
            dataNumBytes = 4
            numParams = 18
            rawData = bytearray(numParams * dataNumBytes)
            strToSend = ''
      
            parsedData = np.zeros(numParams)
            offset = np.zeros(18)
            for x in range(numParams):
                #print(x)
                parsedData[x], = struct.unpack('f', bytesToSend[(x * dataNumBytes):(dataNumBytes + x * dataNumBytes)])
                strToSend += str(parsedData[x]) + ' '
                #print(parsedData[x])
                
            #print(parsedData)
            #print(strToSend)
            pub.publish(strToSend)
            
      
 
    except bluetooth.btcommon.BluetoothError as error:
      print ("Caught BluetoothError: ", error)
      time.sleep(5)
      pc_bluetooth_handle = connect()
      pass
 
    pc_bluetooth_handle.close()
