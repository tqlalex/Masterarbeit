#!/usr/bin/env python3
from operator import itemgetter

import rospy

import pygame

import Wireframe_EKF as wf
import Kalman_EKF as km
import readSensor_EKF as rs
import numpy as np

colors = {
    "x": (255, 0, 0),
    "y": (0, 255, 0),
    "z": (0, 0, 255)
}


class ProjectionViewer:
    """ Displays 3D objects on a Pygame screen """

    def __init__(self, width, height, wireframe, wireframe2 = None):
        self.width = width
        self.height = height
        self.wireframe = wireframe
        self.wireframe2 = wireframe2
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption('Attitude Determination using Quaternions')
        self.background = (180, 180, 180)
        self.clock = pygame.time.Clock()
        pygame.font.init()
        self.font = pygame.font.SysFont('Courier', 20)
        self.data = np.zeros(9)
        self.calibrated = False
        self.ROffset = np.identity(3) # rotation-matrix for calibration
        self.ROffset2 = np.identity(3) # rotation-matrix for calibration

        self.loopRate = 50
        self.avgSensorValues = np.zeros((12, 10)) # 12 variables, 10 values for average

    def run(self, sensorInstance):
        """ Create a pygame screen until it is closed. """
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    sensorInstance.close()
                if event.type == pygame.MOUSEBUTTONDOWN:
                    self.calibrated = True
                    self.ROffset = self.getCalibration()
                    self.ROffset2 = self.getCalibration2()
            self.clock.tick(self.loopRate)
            self.data = sensorInstance.getSerialData()
            """
            k = 0
            for elem in self.data:
                print(k)   
                print(elem)
                k += 1
            """
            self.wireframe.quatRotate([self.data[0], self.data[1], self.data[2]],
                                      [self.data[6], self.data[7], self.data[8]],
                                      [self.data[3], self.data[4], self.data[5]],
                                      1 / self.loopRate)
            self.wireframe2.quatRotate([self.data[0 + 9], self.data[1 + 9], self.data[2 + 9]],
                                      [self.data[6 + 9], self.data[7 + 9], self.data[8 + 9]],
                                      [self.data[3 + 9], self.data[4 + 9], self.data[5 + 9]],
                                      1 / self.loopRate)


            self.display()
            pygame.display.flip()

    def display(self):
        """ Draw the wireframes on the screen. """
        self.screen.fill(self.background)

        # button
        if self.calibrated:
            #text = self.font.render('calibrated', True, (100, 100, 100))
            text = self.font.render('calibrated', True, (0, 255, 255))
            buttonWidth = text.get_width() + 10
            rect = pygame.Rect(((self.screen.get_width() - buttonWidth) / 2, 5), (buttonWidth, 22))
            pygame.draw.rect(self.screen, (255, 255, 255), rect)
            self.screen.blit(text, ((self.screen.get_width() - text.get_width()) / 2 , 0))

        # Get the current attitude
        yaw, pitch, roll = self.wireframe.getAttitude()

        val = self.getOutputValue(yaw, 2)
        str = "Yaw: " + (" " if val >= 0 else "") + "%.1f [deg/s]" % val
        self.messageDisplay(str, self.screen.get_width() - 10, self.screen.get_height() * 0.1, colors["z"], True)

        val = self.getOutputValue(pitch, 1)
        str = "Pitch: " + (" " if val >= 0 else "") + "%.1f [deg/s]" % val
        self.messageDisplay(str, self.screen.get_width() - 10, self.screen.get_height() * 0.05, colors["y"], True)

        val = self.getOutputValue(roll, 0)
        str = "Roll: " + (" " if val >= 0 else "") + "%.1f [deg/s]" % val
        self.messageDisplay(str, self.screen.get_width() - 10, self.screen.get_height() * 0, colors["x"], True)

        # Gyr
        val = self.getOutputValue(self.data[2], 5)
        str = "Gyr_z: " + (" " if val >= 0 else "") + "%.3f [deg/s]" % (val * 180.0 / np.pi)
        self.messageDisplay(str, 10, self.screen.get_height() * 0.1, colors["z"])

        val = self.getOutputValue(self.data[1], 4)
        str = "Gyr_y: " + (" " if val >= 0 else "") + "%.3f [deg/s]" % (val * 180.0 / np.pi)
        self.messageDisplay(str, 10, self.screen.get_height() * 0.05, colors["y"])

        val = self.getOutputValue(self.data[0], 3)
        str = "Gyr_x: " + (" " if val >= 0 else "") + "%.3f [deg/s]" % (val * 180.0 / np.pi)
        self.messageDisplay(str, 10, self.screen.get_height() * 0.0, colors["x"])

        # Acc
        val = self.getOutputValue(self.data[8], 8)
        str = "Acc_z: " + (" " if val >= 0 else "") + "%.3f [m/s^2]" % val
        self.messageDisplay(str, 10, self.screen.get_height(), colors["z"], False, True)

        val = self.getOutputValue(self.data[7], 7)
        str = "Acc_y: " + (" " if val >= 0 else "") + "%.3f [m/s^2]" % val
        self.messageDisplay(str, 10, self.screen.get_height() * 0.95, colors["y"], False, True)

        val = self.getOutputValue(self.data[6], 6)
        str = "Acc_x: " + (" " if val >= 0 else "") + "%.3f [m/s^2]" % val
        self.messageDisplay(str, 10, self.screen.get_height() * 0.9, colors["x"], False, True)

        # Mag
        val = self.getOutputValue(self.data[5], 11)
        str = "Mag_z: " + (" " if val >= 0 else "") + "%.3f [uT]" % val
        self.messageDisplay(str, self.screen.get_width() - 10, self.screen.get_height(), colors["z"], True, True)

        val = self.getOutputValue(self.data[4], 10)
        str = "Mag_y: " + (" " if val >= 0 else "") + "%.3f [uT]" % val
        self.messageDisplay(str, self.screen.get_width() - 10, self.screen.get_height() * 0.95, colors["y"], True, True)

        val = self.getOutputValue(self.data[3], 9)
        str = "Mag_x: " + (" " if val >= 0 else "") + "%.3f [uT]" % val
        self.messageDisplay(str, self.screen.get_width() - 10, self.screen.get_height() * 0.9, colors["x"], True, True)

        # origin of shoulder
        origin = self.getProjection((0, 0, 0))

        # Draw arm
        yElbowRot, yElbow = self.getElbowCoords()
        yElbowTrans = np.matmul(self.ROffset, yElbowRot)
        yElbowProj = self.getProjection(yElbowTrans)
        pygame.draw.line(self.screen, (255,255,0), origin, yElbowProj, 10)  # y


        # Draw second arm
        yWristRot, yWrist = self.getWristCoords()
        yWristTrans = np.matmul(self.ROffset2, yWristRot) + yElbowTrans
        yWristProj = self.getProjection(yWristTrans)
        pygame.draw.line(self.screen, (255,255,0), yElbowProj, yWristProj, 10)

        # match coord to calibrated coord
        xRot, yRot, zRot, x, y, z = self.getCoords()
        xTrans = np.matmul(self.ROffset, xRot)
        yTrans = np.matmul(self.ROffset, yRot)
        zTrans = np.matmul(self.ROffset, zRot)

        xRot2, yRot2, zRot2, x2, y2, z2 = self.getCoords2()
        xTrans2 = np.matmul(self.ROffset2, xRot2) + yElbowTrans
        yTrans2 = np.matmul(self.ROffset2, yRot2) + yElbowTrans
        zTrans2 = np.matmul(self.ROffset2, zRot2) + yElbowTrans

        # Calculate projection of coordinate-system
        #origin = self.getProjection((0, 0, 0))
        xProj = self.getProjection(xTrans)
        yProj = self.getProjection(yTrans)
        zProj = self.getProjection(zTrans)

        xProj2 = self.getProjection2(xTrans2)
        yProj2 = self.getProjection2(yTrans2)
        zProj2 = self.getProjection2(zTrans2)


        # Draw coordinate-system
        pygame.draw.line(self.screen, colors["x"], origin, xProj, 5)  # x
        pygame.draw.line(self.screen, colors["y"], origin, yProj, 5)  # y
        pygame.draw.line(self.screen, colors["z"], origin, zProj, 5)  # z

        pygame.draw.line(self.screen, colors["x"], yElbowProj, xProj2, 5)  # x
        pygame.draw.line(self.screen, colors["y"], yElbowProj, yProj2, 5)  # y
        pygame.draw.line(self.screen, colors["z"], yElbowProj, zProj2, 5)  # z

        # Transform nodes to perspective view
        pvNodes = []
        pvDepth = []
        for node in self.wireframe.nodes:
            pRot = self.getRotation((node.x, node.y, node.z))
            pTrans = np.matmul(self.ROffset, pRot)
            pvNodes.append(self.getProjection(pTrans, pvDepth))
        




        pvNodes2  = []
        pvDepth2 = []
        for node in self.wireframe2.nodes:
            pRot2 = self.getRotation2((node.x, node.y, node.z))
            pTrans2 = np.matmul(self.ROffset2, pRot2) + yElbowTrans
            pvNodes2.append(self.getProjection2(pTrans2, pvDepth2))



        # Calculate the average Z values of each face.
        avg_z = []
        for face in self.wireframe.faces:
            n = pvDepth
            z = (n[face.nodeIndexes[0]] + n[face.nodeIndexes[1]] +
                 n[face.nodeIndexes[2]] + n[face.nodeIndexes[3]]) / 4.0
            avg_z.append(z)

        avg_z2 = []
        for face in self.wireframe2.faces:
            n = pvDepth2
            z = (n[face.nodeIndexes[0]] + n[face.nodeIndexes[1]] +
                 n[face.nodeIndexes[2]] + n[face.nodeIndexes[3]]) / 4.0
            avg_z2.append(z)



        for idx, val in sorted(enumerate(avg_z2), key=itemgetter(1)):
            face = self.wireframe2.faces[idx]
            pointList = [pvNodes2[face.nodeIndexes[0]],
                         pvNodes2[face.nodeIndexes[1]],
                         pvNodes2[face.nodeIndexes[2]],
                         pvNodes2[face.nodeIndexes[3]]]
            pygame.draw.polygon(self.screen, face.color, pointList)

        # Draw the faces using the Painter's algorithm:
        for idx, val in sorted(enumerate(avg_z), key=itemgetter(1)):
            face = self.wireframe.faces[idx]
            pointList = [pvNodes[face.nodeIndexes[0]],
                         pvNodes[face.nodeIndexes[1]],
                         pvNodes[face.nodeIndexes[2]],
                         pvNodes[face.nodeIndexes[3]]]
            pygame.draw.polygon(self.screen, face.color, pointList)





        # Draw arm
        #yElbowRot, yElbow = self.getElbowCoords()
        #yElbowTrans = np.matmul(self.ROffset, yElbowRot)
        #yElbowProj = self.getProjection(yElbowTrans)
        #pygame.draw.line(self.screen, (255,255,0), origin, yElbowProj, 10)  # y

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
        x = (3, 0, 0)
        y = (0, 3, 0)
        z = (0, 0, 3)

        return self.getRotation(x), self.getRotation(y), self.getRotation(z), x, y, z

    def getCoords2(self):
        x = (3, 0, 0)
        y = (0, 3, 0)
        z = (0, 0, 3)

        return self.getRotation2(x), self.getRotation2(y), self.getRotation2(z), x, y, z


    def getElbowCoords(self):
        y = (0, 10, 0)
        return self.getRotation(y),y

    def getWristCoords(self):
        y = (0, 10, 0)
        return self.getRotation2(y),y

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


    def getProjection(self, pos, pvDepth = None):
        if pvDepth is None:
            pvDepth = []

        x, y, z = pos
        xConv, yConv, zConv = self.wireframe.convertToComputerFrame((x, y, z))
        scaling_constant = 70

        # In Pygame, the y axis is downward pointing.
        # In order to make y point upwards, a rotation around x axis by 180 degrees is needed.
        # This will result in y' = -y and z' = -z
        xProjected = xConv * scaling_constant + self.screen.get_width() / 2
        yProjected = -yConv * scaling_constant + self.screen.get_height() / 2
        # Note that there is no negative sign here because our rotation to computer frame
        # assumes that the computer frame is x-right, y-up, z-out
        # so this z-coordinate below is already in the outward direction
        pvDepth.append(zConv)

        return np.round(xProjected), np.round(yProjected)


    def getProjection2(self, pos, pvDepth = None):
        if pvDepth is None:
            pvDepth = []

        x, y, z = pos
        xConv, yConv, zConv = self.wireframe2.convertToComputerFrame((x, y, z))
        scaling_constant = 70

        # In Pygame, the y axis is downward pointing.
        # In order to make y point upwards, a rotation around x axis by 180 degrees is needed.
        # This will result in y' = -y and z' = -z
        xProjected = xConv * scaling_constant + self.screen.get_width() / 2
        yProjected = -yConv * scaling_constant + self.screen.get_height() / 2
        # Note that there is no negative sign here because our rotation to computer frame
        # assumes that the computer frame is x-right, y-up, z-out
        # so this z-coordinate below is already in the outward direction
        pvDepth.append(zConv)

        return np.round(xProjected), np.round(yProjected)

    def messageDisplay(self, text, x, y, color, right = False, bottom = False):
        textSurface = self.font.render(text, True, color, self.background)
        textRect = textSurface.get_rect()
        if (right):
            if (bottom):
                textRect.bottomright = (x, y)
            else:
                textRect.topright = (x, y)
        else:
            if (bottom):
                textRect.bottomleft = (x, y)
            else:
                textRect.topleft = (x, y)
        self.screen.blit(textSurface, textRect)


def initializeCube():
    block = wf.Wireframe()

    block_nodes = [(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-0.1, 0.1)]
    node_colors = [(255, 255, 255)] * len(block_nodes)
    block.addNodes(block_nodes, node_colors)

    faces = [(0, 2, 6, 4), (0, 1, 3, 2), (1, 3, 7, 5), (4, 5, 7, 6), (2, 3, 7, 6), (0, 1, 5, 4)]
    colors = [(255, 0, 255), (255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255), (255, 255, 0)]
    block.addFaces(faces, colors)

    return block


if __name__ == '__main__':
    #s = rs.SerialRead('/dev/cu.usbmodem14201', 115200, 9)
    #s = rs.SerialRead('COM5', 115200, 9)
    #s = rs.SerialRead('COM5', 115200, 18)
    s = rs.SerialRead('/dev/ttyACM0', 115200, 18)
    s.readSerialStart()  # starts background thread

    block = initializeCube()
    block2 = initializeCube()
    #pv = ProjectionViewer(640, 480, block)
    pv = ProjectionViewer(1080, 720, block, block2)
    pv.run(s)


