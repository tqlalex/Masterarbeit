# https://www.thepoorengineer.com/en/quaternion/

import numpy as np


def rad2deg(rad):
    return rad / np.pi * 180


def normalize(arr):
    sum = 0
    for el in arr:
        sum += el ** 2
    sum **= 0.5

    return arr / sum if sum > 0 else 0


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


def getJacobianMatrix(quat, reference):
    q1, q2, q3, q4 = quat
    r1, r2, r3 = reference

    E00 = q1 * r1 + q4 * r2 - q3 * r3
    E01 = q2 * r1 + q3 * r2 + q4 * r3
    E02 = -q3 * r1 + q2 * r2 - q1 * r3
    E03 = -q4 * r1 + q1 * r2 + q2 * r3

    E10 = -q4 * r1 + q1 * r2 + q2 * r3
    E11 = q3 * r1 - q2 * r2 + q1 * r3
    E12 = q2 * r1 + q3 * r2 + q4 * r3
    E13 = -q1 * r1 - q4 * r2 + q3 * r3

    E20 = q3 * r1 - q2 * r2 + q1 * r3
    E21 = q4 * r1 - q1 * r2 - q2 * r3
    E22 = q1 * r1 + q4 * r2 - q3 * r3
    E23 = q2 * r1 + q3 * r2 + q4 * r3

    return 2 * np.array([[E00, E01, E02, E03],
                         [E10, E11, E12, E13],
                         [E20, E21, E22, E23]])


class System:
    def __init__(self, mag_Ainv, mag_b):
        quaternion = np.array([1, 0, 0, 0])  # Initial estimate of the quaternion
        bias = np.zeros(3)  # Initial estimate of the gyro bias

        self.xHatBar = None
        self.xHat = np.concatenate((quaternion, bias))  # array([1, 0, 0, 0, 0, 0, 0])
        self.xHatPrev = None

        self.xHatAcc = np.concatenate((quaternion, bias))



        self.yHatBar = np.zeros(3)
        self.pBar = None

        self.p = np.identity(7) * 0.01  # 7er-Einheitsmatrix * 0.01

        self.C = None

        # Set orientation of axes

        # ENU
        self.accelReference = np.array([0, 0, -1])
        #self.accelReference = np.array([0, 0, -1])
        #self.magReference = np.array([0, 0, 0])  # Disable mag (prev: [0, -1, 0])

        #self.magReference = np.array([0, 0, 0]) 
        #self.magReference = np.array([0, -1, 0])
        #self.magReference = np.array([0.518, 0, 0.855])
        #self.magReference = np.array([0, 0.5578, -0.83])

        # ENU
        
        self.magReference = np.array([0, (1-0.86**2)**0.5, -0.86])
        #self.magReference = np.array([0, 1, 0])
        #self.magReference = np.array([1, 0, 0])

        # todo: Woher kommen diese Werte???
        
        #soft iron distortion
        self.mag_Ainv = mag_Ainv
        """
        self.mag_Ainv = np.array([[2.06423128e-03, -1.04778851e-04, -1.09416190e-06],
                                  [-1.04778851e-04, 1.91693168e-03, 1.79409312e-05],
                                  [-1.09416190e-06, 1.79409312e-05, 1.99819154e-03]])
        """

        #mag1
        
        """
        self.mag_Ainv = np.array([[0.986245358343433, -0.00148132655947, 0.015798738544118],
                                  [-0.00148132655947, 0.993609570591293, 0.011898257863885],
                                  [0.015798738544118, 0.011898257863885, 1.020866111880348]])
        """
        """
        self.mag_Ainv = np.array([[0.998048354867680, 0.008798386099758, 0.007635612896353],
                                  [0.008798386099758, 1.001082158078225, 0.012885399246765],
                                  [0.007635612896353, 0.012885399246765, 1.001172467166756]])

        """
        #mag2
        """
        self.mag_Ainv = np.array([[0.986636367080487, -0.007466754151273, -0.016817474912458],
                                  [-0.007466754151273, 1.009445729557529, 0.022173844964726],
                                  [-0.016817474912458, 0.022173844964726, 1.004884951708161]])
        """
        """
        self.mag_Ainv = np.array([[0.979581534427193, -0.004320265125274, 0.015163204079679],
                                  [-0.004320265125274, 0.982838442098346, 0.013889580190283],
                                  [0.015163204079679, 0.013889580190283, 1.039122291383640]])
        
        """

        # hard iron bias

        #self.mag_b = np.array([80.51340236, 37.08931099, 105.6731885])
        # mag1
        #self.mag_b = np.array([-4.278805689882042, 1.732040755184163, -14.35375926385777])
        #self.mag_b = np.array([-3.623477961268462, 4.179227526598538, -13.836376853190302])        
        # mag2
        #self.mag_b = np.array([-6.974413903655304, -3.006623989339573, -19.605561167368275])
        #self.mag_b = np.array([-4.894215616210016, -2.755887825912465, -16.578810400071870])
        self.mag_b = mag_b

    def getMagVector(self, mag):
        rotMat = getRotMat(self.xHat[0:4])

        #print(mag)
        #print(self.mag_b)
        #print("Mag: ",mag)
        #magLength = (mag[0]**2 + mag[1]**2 + mag[2]**2)**0.5
        #print("magVectorLength: ", magLength)

        #magGaussRaw = np.matmul(np.array(mag) - self.mag_b, self.mag_Ainv)
        #magGaussRaw = np.matmul(mag - self.mag_b, self.mag_Ainv)

        #magGaussRaw[2] = -magGaussRaw[2]

        #print("correctedMagUnitVector: ", normalize(magGaussRaw))
        #print("correctedMag: ", normalize(magGaussRaw))

        #print(magGaussRaw)

        #magGauss_N = np.matmul(rotMat, magGaussRaw)
        #magGauss_N[0] = 0
        #magGauss_N = magGauss_N / (magGauss_N[1] ** 2 + magGauss_N[2] ** 2) ** 0.5

        #print(magGauss_N)
        #print("magvector: ", np.matmul(rotMat.transpose(), magGauss_N))

        return np.matmul(rotMat.transpose(), mag)
        #return np.matmul(rotMat.transpose(), magGauss_N)
    """
    def predictAccelMag(self, predictAccelMag):
        rotMatTrans = getRotMat(self.xHatBar[0:4]).transpose()

        # Accel
        hPrime_a = getJacobianMatrix(self.xHatPrev[0:4], self.accelReference)
        accelBar = np.matmul(rotMatTrans, self.accelReference)

        # Mag
        #hPrime_m = getJacobianMatrix(self.xHatPrev[0:4], self.magReference)
        #magBar = np.matmul(rotMatTrans, self.magReference)

        magReference = np.array([0.0, (1-predictAccelMag**2)**0.5, -predictAccelMag])


        hPrime_m = getJacobianMatrix(self.xHatPrev[0:4], magReference)
        magBar = np.matmul(rotMatTrans, magReference)

        tmp1 = np.concatenate((hPrime_a, np.zeros((3, 3))), axis=1)
        tmp2 = np.concatenate((hPrime_m, np.zeros((3, 3))), axis=1)
        self.C = np.concatenate((tmp1, tmp2), axis=0)

        return np.concatenate((accelBar, magBar), axis=0)
    """

    def predictAccelMag(self):
        rotMatTrans = getRotMat(self.xHatBar[0:4]).transpose()

        # Accel
        hPrime_a = getJacobianMatrix(self.xHatPrev[0:4], self.accelReference)
        accelBar = np.matmul(rotMatTrans, self.accelReference)

        # Mag
        hPrime_m = getJacobianMatrix(self.xHatPrev[0:4], self.magReference)
        magBar = np.matmul(rotMatTrans, self.magReference)

  

        tmp1 = np.concatenate((hPrime_a, np.zeros((3, 3))), axis=1)
        tmp2 = np.concatenate((hPrime_m, np.zeros((3, 3))), axis=1)
        self.C = np.concatenate((tmp1, tmp2), axis=0)

        return np.concatenate((accelBar, magBar), axis=0)



    def predictAccel(self):
        rotMatTrans = getRotMat(self.xHatBar[0:4]).transpose()

        # Accel
        hPrime_a = getJacobianMatrix(self.xHatPrev[0:4], self.accelReference)
        accelBar = np.matmul(rotMatTrans, self.accelReference)


        tmp1 = np.concatenate((hPrime_a, np.zeros((3, 3))), axis=1)

        self.C = temp1

        #return np.concatenate((accelBar, magBar), axis=0)
        return accelBar



    def predict(self, gyr, dt):


        #print("Gyro", gyr)

        q1, q2, q3, q4 = self.xHat[0:4]
        Sq = np.array([[-q2, -q3, -q4],
                       [q1, -q4, q3],
                       [q4, q1, -q2],
                       [-q3, q2, q1]])
        Tmp1 = np.concatenate((
            np.concatenate((np.identity(4), -dt / 2 * Sq), axis=1),
            np.concatenate((np.zeros((3, 4)), np.identity(3)), axis=1)
        ), axis=0)  # [ tmp1, tmp2 ]^T (7 x 7)
        # [[ 1. ,  0. ,  0. ,  0. , s1,  s2,  s3 ],
        #  [ 0. ,  1. ,  0. ,  0. , s4,  s5,  s6 ],
        #  [ 0. ,  0. ,  1. ,  0. , s7,  s8,  s9 ],
        #  [ 0. ,  0. ,  0. ,  1. , s10, s11, s12],
        #  [ 0. ,  0. ,  0. ,  0. ,  1.,  0.,  0.],
        #  [ 0. ,  0. ,  0. ,  0. ,  0.,  1.,  0.],
        #  [ 0. ,  0. ,  0. ,  0. ,  0.,  0.,  1.]]
        Tmp2 = np.concatenate((dt / 2 * Sq, np.zeros((3, 3))), axis=0)  # [ S, 0 ]^T (3 x 7)
        # [-s1,  -s2,  -s3],
        # [-s4,  -s5,  -s6],
        # [-s7,  -s8,  -s9],
        # [-s10, -s11, -s12],
        # [0. , 0. , 0. ],
        # [0. , 0. , 0. ],
        # [0. , 0. , 0. ]]

        gyrox = gyr[0] - self.xHat[4]
        gyroy = gyr[1] - self.xHat[5]
        gyroz = gyr[2] - self.xHat[6]

        #print(gyrox)
        #print(gyroy)
        #print(gyroz)

        #print(gyrox)
        A11 = np.array([[1, -(dt/2) * gyrox, -(dt/2) * gyroy, -(dt/2) * gyroz],
                       [(dt/2) * gyrox, 1, (dt/2) * gyroz, -(dt/2) * gyroy],
                       [(dt/2) * gyroy, -(dt/2) * gyroz, 1, (dt/2) * gyrox],
                       [(dt/2) * gyroz, (dt/2) * gyroy, -(dt/2) * gyrox, 1]])

        jacobiP = np.concatenate((
            np.concatenate((A11, -dt / 2 * Sq), axis=1),
            np.concatenate((np.zeros((3, 4)), np.identity(3)), axis=1)
        ), axis=0)


        self.xHatBar = np.matmul(Tmp1, self.xHat) + np.matmul(Tmp2, np.array(gyr).transpose())
        self.xHatBar[0:4] = normalize(self.xHatBar[0:4])
        self.xHatPrev = self.xHat

        #self.yHatBar = self.predictAccelMag()
        
        #self.pBar = np.matmul(np.matmul(Tmp1, self.p), Tmp1.transpose()) + np.identity(7) * 0.001
        self.pBar = np.matmul(np.matmul(jacobiP, self.p), jacobiP.transpose()) + np.identity(7) * 0.001

    def update(self, a, m):

        self.yHatBar = self.predictAccelMag()

        tmp1 = np.linalg.inv(np.matmul(np.matmul(self.C, self.pBar), self.C.transpose()) + np.identity(6) * 0.1)
        tmp2 = np.matmul(np.matmul(self.pBar, self.C.transpose()), tmp1)

        magGuass_B = self.getMagVector(m)
        #print("magnet: ", m[0], m[1], m[2])
        #print("accel: ", a[0], a[1], a[2])magGuass_B
        measurement = np.concatenate((accel_B, magGuass_B), axis=0)
        self.xHat = self.xHatBar + np.matmul(tmp2, measurement - self.yHatBar)
        self.xHat[0:4] = normalize(self.xHat[0:4])
        self.p = np.matmul(np.identity(7) - np.matmul(tmp2, self.C), self.pBar)



    def updateV2(self, a, m):

        self.yHatBar = self.predictAccelMag()

        tmp1 = np.linalg.inv(np.matmul(np.matmul(self.C, self.pBar), self.C.transpose()) + np.identity(6) * 0.1)
        tmp2 = np.matmul(np.matmul(self.pBar, self.C.transpose()), tmp1)

        #magLength = (mag[0]**2 + mag[1]**2 + mag[2]**2)**0.5
        #print("magVectorLength: ", magLength)

        #print("Mag: ",m)


        #magGuass_B = self.getMagVector(m)

        magGuass_B = normalize(m)


        #magGuass_B = normalize(m)
        #print("zhengchang: ",magGuass_B)
        
        #print("magnet: ", m[0], m[1], m[2])
        #print("accel: ", a[0], a[1], a[2])

        accel_B = normalize(a)

        #print("Acc", accel_B)

        #magGuass_B = np.array([0, 0, 0])

        #print("fanxu: ",magGuass_B[::-1])


        measurement = np.concatenate((accel_B, magGuass_B), axis=0)

        #print(measurement)

        self.xHat = self.xHatBar + np.matmul(tmp2, measurement - self.yHatBar)
        self.xHat[0:4] = normalize(self.xHat[0:4])
        #print(self.xHat[0:4])
        self.p = np.matmul(np.identity(7) - np.matmul(tmp2, self.C), self.pBar)

    def updateAcc(self, a):

        self.yHatBar = self.predictAccel()

        tmp1 = np.linalg.inv(np.matmul(np.matmul(self.C, self.pBar), self.C.transpose()) + np.identity(3) * 0.1)
        tmp2 = np.matmul(np.matmul(self.pBar, self.C.transpose()), tmp1)

        #magGuass_B = self.getMagVector(m)
        #magGuass_B = normalize(m)
        
        #print("magnet: ", m[0], m[1], m[2])
        #print("accel: ", a[0], a[1], a[2])
        accel_B = normalize(a)
        measurement = accel_B

        #measurement = np.concatenate((accel_B, magGuass_B), axis=0)

        q_epsilong = np.matmul(tmp2, measurement - self.yHatBar)
        q_epsilong[3] = 0

        self.xHat = self.xHatBar + q_epsilong
        self.xHat[0:4] = normalize(self.xHat[0:4])
        self.p = np.matmul(np.identity(7) - np.matmul(tmp2, self.C), self.pBar)


