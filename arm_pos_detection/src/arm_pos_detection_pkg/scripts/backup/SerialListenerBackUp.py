#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import numpy as np
import struct

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    strList = data.data.split(" ")
    strList.pop()
    #print(strList)
    rawData = []
    for elem in strList:
        rawData.append(int( '0x'+elem ,16))
    print(rawData)

    rawDataByteArray = bytearray(rawData)

    numParams = 18
    dataNumBytes = 4
    offset = np.zeros(numParams)

    parsedData = np.zeros(numParams)

    for x in range(numParams):
        #print(x)
        parsedData[x], = struct.unpack('f', rawDataByteArray[(x * dataNumBytes):(dataNumBytes + x * dataNumBytes)])
        #print(data[x])

    for j in range(int(numParams/9)):
            
            
        for i in range(9):

            # Gyr
            if i < 3:
                parsedData[i + 9 * j] = (parsedData[i + 9 * j] - offset[i + 9 * j]) / 180.0 * np.pi  # convert from deg/s to rad/s
                #print(j)
                #print(i)
                    
                print(parsedData[i+9*j])

            # Mag - todo: https://www.thepoorengineer.com/en/calibrating-the-magnetometer/
            elif i < 6:
                parsedData[i + 9 * j] -= offset[i + 9 * j]
                #print(j)
                #print(i)
                    
                print(parsedData[i+9*j])
            # Acc
            elif i < 9:
                parsedData[i + 9 * j] -= offset[i + 9 * j]
                #print(j)
                #print(i)
                    
                print(parsedData[i+9*j])
       



def SerialListener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('SerialData', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    SerialListener()