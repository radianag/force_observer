#!/usr/bin/env python
import numpy as np
import math as m

def modified_dh(_dh_alpha, _dh_a, _dh_d, _dh_theta):
    A = np.matrix([[m.cos(_dh_theta), -m.sin(_dh_theta), 0, _dh_a],
    [m.sin(_dh_theta) * m.cos(_dh_alpha), m.cos(_dh_theta) * m.cos(_dh_alpha), -m.sin(_dh_alpha), -m.sin(_dh_alpha) * _dh_d],
    [m.sin(_dh_theta) * m.sin(_dh_alpha), m.cos(_dh_theta) * m.sin(_dh_alpha), m.cos(_dh_alpha), m.cos(_dh_alpha) * _dh_d],
    [0, 0, 0, 1]])

    return A 

yaw =  0
pitch = 1.54

R1 = modified_dh(np.pi/2, 0, 0, yaw + np.pi/2) * modified_dh(-np.pi/2, 0, 0, pitch - np.pi/2)

print(R1)