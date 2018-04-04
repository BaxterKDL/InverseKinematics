# Created on Sat Feb 10 14:51:38 2018
# Authors: Chaitanya Perugu, Ashwin Sahasrabudhe
import numpy as np

def rad2deg(rad):
    return rad*(180.0/np.pi)
    
def deg2rad(deg):
    return deg*(np.pi/180.0)

class Quaternion:
    def __init__(self, val = (1, 0, 0, 0)):
        self.w = val[0]
        self.x = val[1]
        self.y = val[2]
        self.z = val[3]
        self.normalize()
    
    def normalize(self):
        norm = pow(pow(self.w, 2)+pow(self.x, 2)+pow(self.y, 2)+pow(self.z, 2), 0.5)
        self.w = self.w/norm
        self.x = self.x/norm
        self.y = self.y/norm
        self.z = self.z/norm
        
    def q2e(self):
        euler = Euler((0, 0, 0))
        
        sinx = 2.0*(self.w*self.x + self.y*self.z)
        cosx = 1.0 - 2.0*(self.x*self.x + self.y*self.y)
        euler.x = np.arctan2(sinx, cosx)
        
        siny = 2.0*(self.w*self.y - self.z*self.x)
        euler.y = np.arcsin(siny)
        
        sinz = 2.0*(self.w*self.z + self.x*self.y)
        cosz = 1.0 - 2.0*(self.y*self.y + self.z*self.z)
        euler.z = np.arctan2(sinz, cosz)
        
        return euler
        
    def value(self):
        return [self.w, self.x, self.y, self.z]
    
    def display(self):
        print self.value()
        
class Euler:
    def __init__(self, val = (0, 0, 0)):
        self.x = val[0]
        self.y = val[1]
        self.z = val[2]
    
    def e2q(self):
        quat = Quaternion((1, 0, 0, 0))
        
        chz = np.cos(self.z / 2.0)
        shz = np.sin(self.z / 2.0)
        chy = np.cos(self.y / 2.0)
        shy = np.sin(self.y / 2.0)
        chx = np.cos(self.x / 2.0)
        shx = np.sin(self.x / 2.0)
        
        quat.w = chz*chy*chx + shz*shy*shx
        quat.x = shz*chy*chx - chz*shy*shx
        quat.y = chz*shy*chx + shz*chy*shx
        quat.z = chz*chy*shx - shz*shy*chx
        
        return quat
    
    def value(self):
        return [self.x, self.y, self.z]
    
    def display(self):
        print self.value()

class Robot:
    def __init__(self):
        self.lengths = {'l1': 0.27035, 'd1': 0.069, 'l2': 0.36435, 'd2': 0.069,
                        'l3': 0.37429, 'd3': 0.01, 'l4': 0.229525, 'd4': 0}
                        
        self.angle_limits = {'s0': [-141, 51], 's1': [-123, 60], 
                             'e0': [-173.5, 173.5], 'e1': [-3, 150], 
                             'w0': [-175.25, 175.25], 'w1': [-90, 120],
                             'w2': [-175.25, 175.25]}
        
        self.joint_keys = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
        self.length_keys = ['l1', 'd1', 'l2', 'd2', 'l3', 'd3', 'l4', 'd4']
        self.DHref = self.computeDHref()
        
    def computeDHref(self):
        DHtable = {'s0':[self.lengths['l1'], 0, self.lengths['d1'], -90],
                   's1':[0, 90, 0, 90],
                   'e0':[self.lengths['l2'], 0, self.lengths['d2'], -90],
                   'e1':[0, 0, 0, 90],
                   'w0':[self.lengths['l3'], 0, self.lengths['d3'], -90],
                   'w1':[0, 0, 0, 90],
                   'w2':[self.lengths['l4'], 0, self.lengths['d4'], 0]}
        return DHtable

class FKSolver:
    def __init__(self, robot):
        self.robot = robot
        
    def DH2T(self, d, theta, a, alpha):
        # theta = deg2rad(theta)
        # alpha = deg2rad(alpha)
        
        RotZ = np.matrix([[np.cos(theta), -np.sin(theta), 0, 0],
                          [np.sin(theta), np.cos(theta), 0, 0],
                          [0, 0, 1, 0], 
                          [0, 0, 0, 1]])
                         
        TransZ = np.matrix([[1, 0, 0, 0], 
                            [0, 1, 0, 0], 
                            [0, 0, 1, d], 
                            [0, 0, 0, 1]])
                           
        TransX = np.matrix([[1, 0, 0, a], 
                            [0, 1, 0, 0], 
                            [0, 0, 1, 0], 
                            [0, 0, 0, 1]])
                           
        RotX = np.matrix([[1, 0, 0, 0], 
                          [0, np.cos(alpha), -np.sin(alpha), 0],
                          [0, np.sin(alpha), np.cos(alpha), 0],
                          [0, 0, 0, 1]])
        
        T = RotZ*TransZ*TransX*RotX
        return T
        
    def solveFK(self, joint_angles):
        DH = self.robot.DHref
        T = np.asmatrix(np.identity(4))
        
        if self.check_joint_limits(joint_angles):         
            for joint in self.robot.joint_keys:
                T = T*self.DH2T(DH[joint][0], DH[joint][1] + joint_angles[joint], 
                                DH[joint][2], DH[joint][3])
            return np.around(T, 5)
        else:
            print "Invalid FK Request!"
            return 0

    def solveIntermediateFK(self, joint_angles):
        DH = self.robot.DHref
        T = np.asmatrix(np.identity(4))
        # intermediateT = T*len(list(joint_angles.keys()))
        intermediateT = []


        if self.check_joint_limits(joint_angles):
            for joint in self.robot.joint_keys:
                T = T * self.DH2T(DH[joint][0], DH[joint][1] + joint_angles[joint],
                                  DH[joint][2], DH[joint][3])
                intermediateT.append(np.around(T, 5))
            return intermediateT
        else:
            print "Invalid FK Request!"
            return 0
        
    def check_joint_limits(self, joint_angles):
        for joint in joint_angles.keys():
            lower_bound_check = joint_angles[joint] >= self.robot.angle_limits[joint][0]
            upper_bound_check = joint_angles[joint] <= self.robot.angle_limits[joint][1]
            if not (lower_bound_check and upper_bound_check):
                print "{} joint limits exceeded!".format(joint)
                return False
            
        return True