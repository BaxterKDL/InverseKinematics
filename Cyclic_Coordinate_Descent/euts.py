# Created on Sat Feb 10 14:51:38 2018
# Authors: Chaitanya Perugu, Ashwin Sahasrabudhe

#EVERYTHING UNDER THE SUN

import numpy as np
from mathfunctions import *

def rotmat2euler(R):

    if abs(R[2, 0]) != 1:
        theta1 = -np.arcsin(R[2, 0])
        theta2 = np.pi - theta1
        
        psi1 = np.arctan2(R[2, 1]/np.cos(theta1), R[2, 2]/np.cos(theta1))
        psi2 = np.arctan2(R[2, 1]/np.cos(theta2), R[2, 2]/np.cos(theta2))
        
        phi1 = np.arctan2(R[1, 0]/np.cos(theta1), R[0, 0]/np.cos(theta1))
        phi2 = np.arctan2(R[1, 0]/np.cos(theta2), R[0, 0]/np.cos(theta2))
        
        return [rad2deg(psi1), rad2deg(theta1), rad2deg(phi1)]
    else:
        phi = 0
        if R[2, 0] == -1:
            theta = np.pi/2
            psi = phi + np.arctan2(R[0, 1], R[0, 2])
        else:
            theta = -np.pi/2
            psi = -phi + np.arctan2(-R[0, 1], -R[0, 2])
        return [rad2deg(psi), rad2deg(theta), rad2deg(phi)]

def vector2dict(vec, dict_keys):

    if len(dict_keys) != vec.size:
        raise ValueError("Vector and Dictionary don't match!")
    else:
        vec_dict = {}
        for i in range(0, len(dict_keys)):
            vec_dict[dict_keys[i]] = vec.item(i)
        return vec_dict

def add_dicts(a, b):

    if a.keys() != b.keys():
        raise ValueError("Dictionary keys don't match!")
    else:
        c = {}
        for key in a.keys():
            c[key] = a[key] + b[key]
        return c

def dict_euclidean(a, b):
    
    if a.keys() != b.keys():
        raise ValueError("Dictionary keys don't match!")
    else:
        e = 0
        for key in a.keys():
            e += pow(a[key] - b[key], 2)
        return np.sqrt(e)
        