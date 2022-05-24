# https://www.thepoorengineer.com/en/preparing-the-graphics/

import numpy as np
#from scipy.spatial.transform import Rotation as R
#import Kalman_EKF as km
import common_tools_pkg.Kalman_EKF as km


# Node stores each point of the block
class Node:
    def __init__(self, coordinates, color):
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.z = coordinates[2]
        self.color = color


# Face stores 4 nodes that make up a face of the block
class Face:
    def __init__(self, nodes, color):
        self.nodeIndexes = nodes
        self.color = color


# Wireframe stores the details of a block
class Wireframe:
    def __init__(self, mag_Ainv, mag_b):
        self.nodes = []
        self.edges = []
        self.faces = []
        
        self.mag_Ainv = mag_Ainv
        self.mag_b = mag_b

        self.sys = km.System(mag_Ainv, mag_b)

    def addNodes(self, nodeList, colorList):
        for node, color in zip(nodeList, colorList):
            self.nodes.append(Node(node, color))

    def addFaces(self, faceList, colorList):
        for indexes, color in zip(faceList, colorList):
            self.faces.append(Face(indexes, color))

    def quatRotate(self, acc, gyr, mag, dt):
        self.sys.predict(gyr, dt)
        #self.sys.update(acc, mag)
        self.sys.updateV2(acc, mag)
    """
    def quatRotate(self, acc, gyr, mag, dt, earthMagDown):
        self.sys.predict(gyr, dt)
        #self.sys.update(acc, mag)
        self.sys.updateV2(acc, mag, earthMagDown)
    """

    def rotatePoint(self, point):
        rotationMat = km.getRotMat(self.sys.xHat[0:4])

        return np.matmul(rotationMat, point)

    def convertToComputerFrame(self, point):
        computerFrameChangeMatrix = np.array([[-1, 0, 0], [0, 0, -1], [0, -1, 0]])

        return np.matmul(computerFrameChangeMatrix, point)

    def getAttitude(self):
        return km.getEulerAngles(self.sys.xHat[0:4])

        

