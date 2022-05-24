#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import struct
import math
from tf import TransformBroadcaster

import common_tools_pkg.Wireframe_EKF as wf
import common_tools_pkg.Kalman_EKF as km
import common_tools_pkg.readSensor_EKF as rs
#import Wireframe_EKF as wf
#import Kalman_EKF as km
#import readSensor_EKF as rs


def normalize(arr):
    sum = 0
    for el in arr:
        sum += el ** 2
    sum **= 0.5

    return arr / sum if sum > 0 else 0





"""
def SerialListener(block1, block2):

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('SerialData', String, callback, (block1, block2))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
"""

def getRotMat(quat):
    # https://www.cbcity.de/tutorial-rotationsmatrix-und-quaternion-einfach-erklaert-in-din70000-zyx-konvention
    q1, q2, q3, q4 = quat

    R11 = q1 ** 2 + q2 ** 2 - q3 ** 2 - q4 ** 2
    R12 = 2 * (q2 * q3 - q1 * q4)
    R13 = 2 * (q2 * q4 + q1 * q3)

    R21 = 2 * (q2 * q3 + q1 * q4)
    R22 = q1 ** 2 - q2 ** 2 + q3 ** 2 - q4 ** 2
    R23 = 2 * (q3 * q4 - q1 * q2)

    R31 = 2 * (q2 * q4 - q1 * q3)
    R32 = 2 * (q3 * q4 + q1 * q2)
    R33 = q1 ** 2 - q2 ** 2 - q3 ** 2 + q4 ** 2

    return np.array([[R11, R12, R13], [R21, R22, R23], [R31, R32, R33]])


def getEulerAngles(quat):
    rot = getRotMat(quat)
    threshold = 0.99999  # gimballock-Threshold

    if rot[2, 0] < -threshold:  # R31
        yaw = 0
        pitch = np.pi / 2
        roll = np.arctan2(rot[0, 1], rot[0, 2])  # R12, R13
    elif rot[2, 0] > threshold:  # R31
        yaw = 0
        pitch = -np.pi / 2
        roll = np.arctan2(-rot[0, 1], -rot[0, 2])  # R12, R13
    else:
        yaw = np.arctan2(rot[1, 0], rot[0, 0])  # R21, R11
        pitch = np.arcsin(-rot[2, 0])  # R31
        roll = np.arctan2(rot[2, 1], rot[2, 2])  # R32, R33

    return yaw, pitch, roll
    #return rad2deg(yaw), rad2deg(pitch), rad2deg(roll)


def quatMultiplication(quat1, quat2):
    q0 = quat1[0] * quat2[0] - quat1[1] * quat2[1] - quat1[2] * quat2[2] - quat1[3] * quat2[3]
    q1 = quat1[1] * quat2[0] + quat1[0] * quat2[1] - quat1[3] * quat2[2] + quat1[2] * quat2[3]
    q2 = quat1[2] * quat2[0] + quat1[3] * quat2[1] + quat1[0] * quat2[2] - quat1[1] * quat2[3]
    q3 = quat1[3] * quat2[0] - quat1[2] * quat2[1] + quat1[1] * quat2[2] + quat1[0] * quat2[3]
    return np.array([q0, q1, q2, q3])

def quatReverse(quat):
    q0, q1, q2, q3 = quat
    return np.array([q0, -q1, -q2, -q3])



def rotToEulerAngles(rot):
    threshold = 0.99999  # gimballock-Threshold

    if rot[2, 0] < -threshold:  # R31
        yaw = 0
        pitch = np.pi / 2
        roll = np.arctan2(rot[0, 1], rot[0, 2])  # R12, R13
    elif rot[2, 0] > threshold:  # R31
        yaw = 0
        pitch = -np.pi / 2
        roll = np.arctan2(-rot[0, 1], -rot[0, 2])  # R12, R13
    else:
        yaw = np.arctan2(rot[1, 0], rot[0, 0])  # R21, R11
        pitch = np.arcsin(-rot[2, 0])  # R31
        roll = np.arctan2(rot[2, 1], rot[2, 2])  # R32, R33

    return yaw, pitch, roll
    #return rad2deg(yaw), rad2deg(pitch), rad2deg(roll)



def rotationMatrixToEulerAngles(R) :

    #assert(isRotationMatrix(R))
    
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    
    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return x, y, z

class SubscribeAndPublish:
    def __init__(self, block, block2):
        self.wireframe = block
        self.wireframe2 = block2
        self.loopRate = 50
        #self.loopRate = 100
        self.calibrated = False
        self.ROffset = np.identity(3) # rotation-matrix for calibration
        self.quatOffset = np.array([1, 0, 0, 0])
        self.ROffset2 = np.identity(3) # rotation-matrix for calibration
        self.quatOffset2 = np.array([1, 0, 0, 0])
        rospy.init_node('Serial_Joint_State_Publisher', anonymous=True)
        self.serialSubscriber = rospy.Subscriber('key', Int8, self.callbackKeyBoard)
        self.serialSubscriber = rospy.Subscriber('SerialData', String, self.callback)
        self.jointStatePublisher = rospy.Publisher('joint_states', JointState, queue_size=10)
        
        self.shoulderBroadcaster = TransformBroadcaster()
        self.elbowBroadcaster = TransformBroadcaster()

        self.magVerticalSpeicher = []





    def getOutputValue(self, value, index):
        sum = 0
        length = len(self.avgSensorValues[index])

        for i in range(length):
            if i < length - 1:
                self.avgSensorValues[index][i] = self.avgSensorValues[index][i + 1]
            else:
                self.avgSensorValues[index][i] = value
            sum += self.avgSensorValues[index][i]

        return sum / length

    def getRotation(self, pos):
        x, y, z = pos

        return self.wireframe.rotatePoint((x, y, z))

    def getRotation2(self, pos):
        x, y, z = pos

        return self.wireframe2.rotatePoint((x, y, z))


    def getCoords(self):
        x = (1, 0, 0)
        y = (0, 1, 0)
        z = (0, 0, 1)

        return self.getRotation(x), self.getRotation(y), self.getRotation(z), x, y, z

    def getCoords2(self):
        x = (1, 0, 0)
        y = (0, 1, 0)
        z = (0, 0, 1)

        return self.getRotation2(x), self.getRotation2(y), self.getRotation2(z), x, y, z


    def getCalibration(self):
        xRot, yRot, zRot, x, y, z = self.getCoords()
        CoordIst = np.array([xRot, yRot, zRot])
        CoordSoll = np.array([x, y, z])

        M = np.array([[0., 0., 0.],
                      [0., 0., 0.],
                      [0., 0., 0.]])

        for i, coord1 in enumerate(CoordIst):
            M += np.outer(coord1, CoordSoll[i])

        N00 = M[0, 0] + M[1, 1] + M[2, 2]
        N01 = M[1, 2] - M[2, 1]
        N02 = M[2, 0] - M[0, 2]
        N03 = M[0, 1] - M[1, 0]

        N10 = N01
        N11 = M[0, 0] - M[1, 1] - M[2, 2]
        N12 = M[0, 1] + M[1, 0]
        N13 = M[2, 0] + M[0, 2]

        N20 = N02
        N21 = N12
        N22 = -M[0, 0] + M[1, 1] - M[2, 2]
        N23 = M[1, 2] + M[2, 1]

        N30 = N03
        N31 = N13
        N32 = N23
        N33 = -M[0, 0] - M[1, 1] + M[2, 2]

        N = np.array([[N00, N01, N02, N03],
                      [N10, N11, N12, N13],
                      [N20, N21, N22, N23],
                      [N30, N31, N32, N33]])

        values, vectors = np.linalg.eig(N)
        w = list(values)
        mw = max(w)

        return km.getRotMat(np.array(vectors[:, w.index(mw)]))

    def getCalibrationQuat(self):
        xRot, yRot, zRot, x, y, z = self.getCoords()
        CoordIst = np.array([xRot, yRot, zRot])
        CoordSoll = np.array([x, y, z])

        M = np.array([[0., 0., 0.],
                      [0., 0., 0.],
                      [0., 0., 0.]])

        for i, coord1 in enumerate(CoordIst):
            M += np.outer(coord1, CoordSoll[i])

        N00 = M[0, 0] + M[1, 1] + M[2, 2]
        N01 = M[1, 2] - M[2, 1]
        N02 = M[2, 0] - M[0, 2]
        N03 = M[0, 1] - M[1, 0]

        N10 = N01
        N11 = M[0, 0] - M[1, 1] - M[2, 2]
        N12 = M[0, 1] + M[1, 0]
        N13 = M[2, 0] + M[0, 2]

        N20 = N02
        N21 = N12
        N22 = -M[0, 0] + M[1, 1] - M[2, 2]
        N23 = M[1, 2] + M[2, 1]

        N30 = N03
        N31 = N13
        N32 = N23
        N33 = -M[0, 0] - M[1, 1] + M[2, 2]

        N = np.array([[N00, N01, N02, N03],
                      [N10, N11, N12, N13],
                      [N20, N21, N22, N23],
                      [N30, N31, N32, N33]])

        values, vectors = np.linalg.eig(N)
        w = list(values)
        mw = max(w)

        return np.array(vectors[:, w.index(mw)])
        #return km.getRotMat(np.array(vectors[:, w.index(mw)]))



    def getCalibration2(self):
        xRot, yRot, zRot, x, y, z = self.getCoords2()
        CoordIst = np.array([xRot, yRot, zRot])
        CoordSoll = np.array([x, y, z])

        M = np.array([[0., 0., 0.],
                      [0., 0., 0.],
                      [0., 0., 0.]])

        for i, coord1 in enumerate(CoordIst):
            M += np.outer(coord1, CoordSoll[i])

        N00 = M[0, 0] + M[1, 1] + M[2, 2]
        N01 = M[1, 2] - M[2, 1]
        N02 = M[2, 0] - M[0, 2]
        N03 = M[0, 1] - M[1, 0]

        N10 = N01
        N11 = M[0, 0] - M[1, 1] - M[2, 2]
        N12 = M[0, 1] + M[1, 0]
        N13 = M[2, 0] + M[0, 2]

        N20 = N02
        N21 = N12
        N22 = -M[0, 0] + M[1, 1] - M[2, 2]
        N23 = M[1, 2] + M[2, 1]

        N30 = N03
        N31 = N13
        N32 = N23
        N33 = -M[0, 0] - M[1, 1] + M[2, 2]

        N = np.array([[N00, N01, N02, N03],
                      [N10, N11, N12, N13],
                      [N20, N21, N22, N23],
                      [N30, N31, N32, N33]])

        values, vectors = np.linalg.eig(N)
        w = list(values)
        mw = max(w)

        return km.getRotMat(np.array(vectors[:, w.index(mw)]))

    def getCalibrationQuat2(self):
        xRot, yRot, zRot, x, y, z = self.getCoords2()
        CoordIst = np.array([xRot, yRot, zRot])
        CoordSoll = np.array([x, y, z])

        M = np.array([[0., 0., 0.],
                      [0., 0., 0.],
                      [0., 0., 0.]])

        for i, coord1 in enumerate(CoordIst):
            M += np.outer(coord1, CoordSoll[i])

        N00 = M[0, 0] + M[1, 1] + M[2, 2]
        N01 = M[1, 2] - M[2, 1]
        N02 = M[2, 0] - M[0, 2]
        N03 = M[0, 1] - M[1, 0]

        N10 = N01
        N11 = M[0, 0] - M[1, 1] - M[2, 2]
        N12 = M[0, 1] + M[1, 0]
        N13 = M[2, 0] + M[0, 2]

        N20 = N02
        N21 = N12
        N22 = -M[0, 0] + M[1, 1] - M[2, 2]
        N23 = M[1, 2] + M[2, 1]

        N30 = N03
        N31 = N13
        N32 = N23
        N33 = -M[0, 0] - M[1, 1] + M[2, 2]

        N = np.array([[N00, N01, N02, N03],
                      [N10, N11, N12, N13],
                      [N20, N21, N22, N23],
                      [N30, N31, N32, N33]])

        values, vectors = np.linalg.eig(N)
        w = list(values)
        mw = max(w)

        return np.array(vectors[:, w.index(mw)])
        #return km.getRotMat(np.array(vectors[:, w.index(mw)]))


    def callbackKeyBoard(self, data):
        print("I heard: ")
        print(data)
        print(type(data))
        if data.data == 65:
            self.calibrated = True

    def callback(self, data):

        #print(self.calibrated)

        if self.calibrated == True:  
            self.ROffset = self.getCalibration()
            self.ROffset2 = self.getCalibration2()
            self.quatOffset = self.getCalibrationQuat()
            self.quatOffset2 = self.getCalibrationQuat2()
            self.calibrated = False


        strList = data.data.split(" ")
        strList.pop()
        #print(strList)
        rawData = []
        for elem in strList:
            rawData.append(int( '0x'+elem ,16))
        #print(rawData)

        rawDataByteArray = bytearray(rawData)

        numParams = 18
        dataNumBytes = 4
        offset = np.zeros(numParams)

        parsedData = np.zeros(numParams)

        for x in range(numParams):
            #print(x)
            parsedData[x], = struct.unpack('f', rawDataByteArray[(x * dataNumBytes):(dataNumBytes + x * dataNumBytes)])
            #print(parsedData[x])

        for j in range(int(numParams/9)):
            
            
            for i in range(9):

                # Acc
                # Gyr
                if i < 3:

                    parsedData[i + 9 * j] -= offset[i + 9 * j]
                    #parsedData[i + 9 * j] = (parsedData[i + 9 * j] - offset[i + 9 * j]) / 180.0 * np.pi  # convert from deg/s to rad/s
                    #print(j)
                    #print(i)
                    
                    #print(parsedData[i+9*j])

                # Gyro
                # Mag - todo: https://www.thepoorengineer.com/en/calibrating-the-magnetometer/
                elif i < 6:

                    parsedData[i + 9 * j] = (parsedData[i + 9 * j] - offset[i + 9 * j]) / 180.0 * np.pi
                    #parsedData[i + 9 * j] -= offset[i + 9 * j]
                    #print(j)
                    #print(i)
                    
                    #print(parsedData[i+9*j])
                # Acc
                elif i < 9:
                    parsedData[i + 9 * j] -= offset[i + 9 * j]
                    #print(j)
                    #print(i)
                    
                    #print(parsedData[i+9*j])

        """            
        print("begin")
        for elem in parsedData:
            print(elem)
        print("end")
        """





        ########################  Senror1 ###############################



        acc = np.array([parsedData[1], -parsedData[0], -parsedData[2]])
        gyro = np.array([-parsedData[4], parsedData[3], parsedData[5]])
        mag = np.array([parsedData[6], parsedData[7],parsedData[8]])


        acc = normalize(acc)

        if acc[2] <= 0:
            w = ((1-acc[2])/2)**0.5
            x = -acc[1]/((2*(1 - acc[2]))**0.5)
            y = acc[0]/((2*(1 - acc[2]))**0.5)
            z = 0
        else:
            w = -acc[1]/((2*(1 + acc[2]))**0.5)
            x = ((1 + acc[2])/2)**0.5
            y = 0
            z = -acc[0]/((2*(1 + acc[2]))**0.5)


        rotationShoulder = (x, y, z, w)
        rotationShoulderNp = np.array([w, x, y, z])
        #print("rotationElblowNp: ",rotationElbowNp)
        rotationShoulderNpReverse = quatReverse(rotationShoulderNp)




        #mag1_Cali
        mag_Ainv = np.array([[0.995245167640106, -0.00899834725534734, 0.00160962053300617],
                         [-0.00899834725534734, 0.98946883290268, 0.0147716094005681],
                         [0.00160962053300617, 0.0147716094005681, 1.01577873139347]])
        mag_b = np.array([-4.25221227063314, 4.48360900748122, -14.0807867417639])  
    

        magGaussRaw = np.matmul(mag - mag_b, mag_Ainv)

      
        normalizedMag = normalize(magGaussRaw)


        normalizedMagENU = np.array([normalizedMag[1], normalizedMag[0], -normalizedMag[2]])
   
        
        magVectorPureQuat = np.array([0, normalizedMagENU[0], normalizedMagENU[1], normalizedMagENU[2]]) 
        

        magHoriz = quatMultiplication(rotationShoulderNp, quatMultiplication(magVectorPureQuat, rotationShoulderNpReverse))
       

        magHorizVector = np.array([magHoriz[1],magHoriz[2],magHoriz[3]])

        earthMagDown = magHoriz[3]
  


        """        
        if len(self.magVerticalSpeicher) <= 100:
            self.magVerticalSpeicher.append(magHoriz2[3])

        if len(self.magVerticalSpeicher) == 100:
            self.magVerticalSpeicher.pop(0)

        
        #print(self.magVerticalSpeicher)
        


        sumMagVer2 = 0
        for i in range(len(self.magVerticalSpeicher)):
            sumMagVer2 += self.magVerticalSpeicher[i]
        print("Average Vertical Part: ", sumMagVer2/len(self.magVerticalSpeicher))
        """



        gravityVectorPureQuat = np.array([0, acc[0], acc[1], acc[2]])
        #print("Acc:", acc[0], acc[1], acc[2])
        gravityHoriz = quatMultiplication(rotationShoulderNp, quatMultiplication(gravityVectorPureQuat, rotationShoulderNpReverse))
        #print("gravityHoriz: ", gravityHoriz)


        



        #self.wireframe.quatRotate(acc1ToFilter, gyro1ToFilter, mag1toFilter, 1 / self.loopRate)
        self.wireframe.quatRotate(acc, gyro, normalizedMagENU, 1 / self.loopRate)



    
        quatBeforeCali = self.wireframe.sys.xHat[0:4]
        quatAfterCali = quatMultiplication(quatBeforeCali, self.quatOffset)
        #print("2: ", quatAfterCali2)

        
        #print("1: ", quatAfterCaliReverse)        
        
        #print("3: ", quatAfterCali2)

        q0_shoulder, q1_shoulder, q2_shoulder, q3_shoulder = quatAfterCali
        rotationShoulder = (q1_shoulder, q2_shoulder, q3_shoulder, q0_shoulder)
       
  
        translationShoulder = (1.0, 0.0, 0.0)

        self.elbowBroadcaster.sendTransform(translationShoulder, rotationShoulder, rospy.Time.now(), 'upper_arm', '/base')






        ################## Sensor2 ######################

        #ENU

        acc2 = np.array([parsedData[1+9], -parsedData[0+9], -parsedData[2+9]])
        gyro2 = np.array([-parsedData[4+9], parsedData[3+9], parsedData[5+9]])
        #print("gyro2: ", gyro2)
        mag2 = np.array([parsedData[6+9], parsedData[7+9],parsedData[8+9]])



        #sum = ( parsedData[1+9]**2 + parsedData[0+9]**2 + parsedData[2+9]**2)**0.5
        #print("SumAcc: ", sum)

        acc2 = normalize(acc2)
        #print("acc22222222222222222: ", acc2)
        #print("Gyro1: ", gyro1)
        #print("Mag2: ", mag2)

        if acc2[2] <= 0:
            w2 = ((1-acc2[2])/2)**0.5
            x2 = -acc2[1]/((2*(1 - acc2[2]))**0.5)
            y2 = acc2[0]/((2*(1 - acc2[2]))**0.5)
            z2 = 0
        else:
            w2 = -acc2[1]/((2*(1 + acc2[2]))**0.5)
            x2 = ((1 + acc2[2])/2)**0.5
            y2 = 0
            z2 = -acc2[0]/((2*(1 + acc2[2]))**0.5)


        rotationElbow = (x2, y2, z2, w2)
        rotationElbowNp = np.array([w2, x2, y2, z2])
        #print("rotationElblowNp: ",rotationElbowNp)
        rotationElbowNpReverse = quatReverse(rotationElbowNp)


        """
        mag_Ainv2 = np.array([[2.06423128e-03, -1.04778851e-04, -1.09416190e-06],
                                  [-1.04778851e-04, 1.91693168e-03, 1.79409312e-05],
                                  [-1.09416190e-06, 1.79409312e-05, 1.99819154e-03]])

        mag_b2 = np.array([80.51340236, 37.08931099, 105.6731885])
        """
        mag_Ainv2 = np.array([[0.998180693422652, -0.00350389605393174, -0.0071810150496093],
                                  [-0.00350389605393174, 1.00016298876368, 0.000865214579243101],
                                  [-0.0071810150496093, 0.000865214579243101, 1.00172404795303]])

        mag_b2 = np.array([-4.66915483530062, -3.2401404010871, -18.0489330259729])


        """
        mag_Ainv2 = np.array([[0.979581534427193, -0.004320265125274, 0.015163204079679],
                         [-0.004320265125274, 0.982838442098346, 0.013889580190283],
                         [0.015163204079679, 0.013889580190283, 1.039122291383640]])
        mag_b2 = np.array([-4.894215616210016, -2.755887825912465, -16.578810400071870])
        """
        magGaussRaw2 = np.matmul(mag2 - mag_b2, mag_Ainv2)

        #print("magGaussRaw2: ", magGaussRaw2)

        normalizedMag2 = normalize(magGaussRaw2)

        #print("NormalizedMag2: ", normalize(magGaussRaw2))


        normalizedMagENU2 = np.array([normalizedMag2[1], normalizedMag2[0], -normalizedMag2[2]])
        #print("normalizedMagENU2", normalizedMagENU2)
        #sum = (magGaussRaw2[0]**2 + magGaussRaw2[1]**2 + magGaussRaw2[2]**2)**0.5
        #print("Sum Gauss: " , sum)

       

        
        normalizedMagCali2 = normalize(magGaussRaw2)
        #print("normalizedMagCali2", normalizedMagCali2)

        
        magVectorPureQuat2 = np.array([0, normalizedMagENU2[0], normalizedMagENU2[1], normalizedMagENU2[2]]) 
        #print("MagENU2:", normalizedMagENU2[0], normalizedMagENU2[1], normalizedMagENU2[2])
        magHoriz2 = quatMultiplication(rotationElbowNp, quatMultiplication(magVectorPureQuat2, rotationElbowNpReverse))
        #print("magHoriz2: ", magHoriz2[1], magHoriz2[2], magHoriz2[3] )

        magHorizVector2 = np.array([magHoriz2[1],magHoriz2[2],magHoriz2[3]])

        earthMagDown2 = magHoriz2[3]
        #print("earthMagDown2", earthMagDown2)


        """        
        if len(self.magVerticalSpeicher) <= 100:
            self.magVerticalSpeicher.append(magHoriz2[3])

        if len(self.magVerticalSpeicher) == 100:
            self.magVerticalSpeicher.pop(0)

        
        #print(self.magVerticalSpeicher)
        


        sumMagVer2 = 0
        for i in range(len(self.magVerticalSpeicher)):
            sumMagVer2 += self.magVerticalSpeicher[i]
        print("Average Vertical Part: ", sumMagVer2/len(self.magVerticalSpeicher))
        """



        gravityVectorPureQuat2 = np.array([0, acc2[0], acc2[1], acc2[2]])
        #print("Acc2:", acc2[0], acc2[1], acc2[2])
        gravityHoriz2 = quatMultiplication(rotationElbowNp, quatMultiplication(gravityVectorPureQuat2, rotationElbowNpReverse))
        #print("gravityHoriz2: ", gravityHoriz2)



        translationElbow = (2.0, 0.0, 0.0)

 


        #NED
        """
        if acc1[2] >= 0:
            w = ((acc1[2] + 1) / 2) ** 0.5 
            x = -acc1[1]/((2*(acc1[2] + 1))**0.5)
            y = -acc1[0]/((2*(acc1[2] + 1))**0.5)
            z = 0
        else:
            w = -acc1[1]/((2*(1 - acc1[2]))**0.5)
            x = ((1 - acc1[2])/2)**0.5
            y = 0
            z = acc1[0]/((2*(1 - acc1[2]))**0.5)
        """



        
    



        self.wireframe2.quatRotate(acc2, gyro2, normalizedMagENU2, 1 / self.loopRate)



        quatBeforeCali = self.wireframe.sys.xHat[0:4]
    
        ######### wire frame2 ###########
        #print("quatBeforCali", quatBeforeCali)
        quatBeforeCali2 = self.wireframe2.sys.xHat[0:4]
        quatAfterCali2 = quatMultiplication(quatBeforeCali2, self.quatOffset2)
        #print("2: ", quatAfterCali2)

        
        #print("1: ", quatAfterCaliReverse)        
        
        #print("3: ", quatAfterCali2)
        quatAfterCaliReverse = quatReverse(quatAfterCali)
        quatAfterCali2 = quatMultiplication(quatAfterCaliReverse, quatAfterCali2)

        q0_elbow, q1_elbow, q2_elbow, q3_elbow = quatAfterCali2

        




        rotationElbow = (q1_elbow, q2_elbow, q3_elbow, q0_elbow)
        #translationElbow = (1.0, 0.0, 0.0)
  
        translationElbow = (1.0, 0.0, 0.0)

        self.elbowBroadcaster.sendTransform(translationElbow, rotationElbow, rospy.Time.now(), 'lower_arm', 'upper_arm')



        
        #print("rollShoulder: ", rollShoulder, "pitchShoulder: ", pitchShoulder)

        yawElbow, pitchElbow, rollElbow = getEulerAngles(quatAfterCali2)
        #print("rollElbow: ", rollElbow, "pitchElbow: ", pitchElbow)






        #quatAfterCali = quatMultiplication(self.quatOffset,quatBeforeCali)

        
        quatAfterCali = quatMultiplication(quatBeforeCali, self.quatOffset)
        #quatAfterCaliReverse = quatReverse(quatAfterCali)
        
        quatAfterCali2 = quatMultiplication(quatAfterCaliReverse, quatAfterCali2)
        #print("0: ", quatAfterCali)
        #print(self.quatOffset)
        yawShoulder, pitchShoulder, rollShoulder = getEulerAngles(quatAfterCali)
        q0_shoulder, q1_shoulder, q2_shoulder, q3_shoulder = quatAfterCali
        rotationShoulderQuat = (q1_shoulder, q2_shoulder, q3_shoulder, q0_shoulder)

        #self.shoulderBroadcaster.sendTransform(translationShoulder, rotationShoulderQuat, rospy.Time.now(), 'upper_arm', '/base' )

        #yawShoulder, pitchShoulder, rollShoulder  = rotToEulerAngles(rotMatWF)
        #yawElbow, pitchElbow, rollElbow = rotToEulerAngles(rotMatWF2)
        #rollShoulder, pitchShoulder, yawShoulder  = rotationMatrixToEulerAngles(rotMatWF.transpose())
        #rollElbow, pitchElbow, yawElbow = rotationMatrixToEulerAngles(rotMatWF2.transpose())

        #yawShoulder, pitchShoulder, rollShoulder = self.wireframe.getAttitude()
        #print(yawShoulder, pitchShoulder, rollShoulder)
        #yawElbow, pitchElbow, rollElbow = self.wireframe2.getAttitude()

        #yawShoulder, pitchShoulder, rollShoulder = self.wireframe.getAttitudeAfterCalibration(self.ROffset)
        #yawElbow, pitchElbow, rollElbow = self.wireframe2.getAttitudeAfterCalibration(self.ROffset2)

        jointStateMsg = JointState()
        jointStateMsg.header = Header()
        jointStateMsg.header.stamp = rospy.Time.now()
        jointStateMsg.name = ['shoulder_yaw', 'shoulder_pitch', 'shoulder_roll', 'elbow_yaw', 'elbow_pitch', 'elbow_roll']
        jointStateMsg.position = [yawShoulder, pitchShoulder, rollShoulder, yawElbow - yawShoulder, pitchElbow - pitchShoulder, rollElbow - rollShoulder]
        #jointStateMsg.position = [yawShoulder, pitchShoulder, rollShoulder, -yawShoulder, -pitchShoulder, -rollShoulder]
        jointStateMsg.velocity = []
        jointStateMsg.effort = []

        #self.jointStatePublisher.publish(jointStateMsg)
        






        
def initializeCube(mag_Ainv, mag_b):
    block = wf.Wireframe(mag_Ainv, mag_b)
    block_nodes = [(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-0.1, 0.1)]
    node_colors = [(255, 255, 255)] * len(block_nodes)
    block.addNodes(block_nodes, node_colors)

    faces = [(0, 2, 6, 4), (0, 1, 3, 2), (1, 3, 7, 5), (4, 5, 7, 6), (2, 3, 7, 6), (0, 1, 5, 4)]
    colors = [(255, 0, 255), (255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255), (255, 255, 0)]
    block.addFaces(faces, colors)

    return block 



if __name__ == '__main__':

    mag_Ainv = np.array([[0.998048354867680, 0.008798386099758, 0.007635612896353],
                         [0.008798386099758, 1.001082158078225, 0.012885399246765],
                         [0.007635612896353, 0.012885399246765, 1.001172467166756]])
    mag_b = np.array([-3.623477961268462, 4.179227526598538, -13.836376853190302])  

    mag_Ainv2 = np.array([[0.979581534427193, -0.004320265125274, 0.015163204079679],
                         [-0.004320265125274, 0.982838442098346, 0.013889580190283],
                         [0.015163204079679, 0.013889580190283, 1.039122291383640]])
    mag_b2 = np.array([-4.894215616210016, -2.755887825912465, -16.578810400071870])


    block = initializeCube(mag_Ainv, mag_b)
    #block = initializeCube()
    block2 = initializeCube(mag_Ainv2, mag_b2)
    #SerialListener(block, block2)
    SubscribeAndPublish(block,block2)

    rospy.spin()


