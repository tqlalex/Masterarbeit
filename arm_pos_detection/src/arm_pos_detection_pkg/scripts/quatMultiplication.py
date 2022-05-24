#!/usr/bin/env python3

import numpy as np

def quatMultiplication(quat1, quat2):
    q0 = quat1[0] * quat2[0] - quat1[1] * quat2[1] - quat1[2] * quat2[2] - quat1[3] * quat2[3]
    q1 = quat1[1] * quat2[0] + quat1[0] * quat2[1] - quat1[3] * quat2[2] + quat1[2] * quat2[3]
    q2 = quat1[2] * quat2[0] + quat1[3] * quat2[1] + quat1[0] * quat2[2] - quat1[1] * quat2[3]
    q3 = quat1[3] * quat2[0] - quat1[2] * quat2[1] + quat1[1] * quat2[2] + quat1[0] * quat2[3]
    return np.array([q0, q1, q2, q3])
    
    
def quatReverse(quat):
    q0, q1, q2, q3 = quat
    return np.array([q0, -q1, -q2, -q3])


qRot = np.array([0.707, 0, 0, (1-0.707**2)**0.5])
vector = np.array([0, 1, 0, 0])

vectorAfterTrans = quatMultiplication(quatReverse(qRot), quatMultiplication(vector, qRot))

vectorAfterRot = quatMultiplication(qRot, quatMultiplication(vector, quatReverse(qRot)))

print("original vector: ", vector)

print("Trans: ", vectorAfterTrans)

print("Rot: ",vectorAfterRot )
