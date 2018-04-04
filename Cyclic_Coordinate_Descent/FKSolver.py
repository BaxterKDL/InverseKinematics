# Created on Sat Feb 10 14:51:38 2018
# Authors: Chaitanya Perugu, Ashwin Sahasrabudhe

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mathfunctions import *
from euts import *

class FKSolver:
    def __init__(self, robot):
        self.robot = robot
        
    def DH2T(self, d, theta, a, alpha):
        theta = deg2rad(theta)
        alpha = deg2rad(alpha)
        
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
        
    def solveFK(self, joint_angles, visualize=False, output='t'):
        DH = self.robot.DHref
        T = np.asmatrix(np.identity(4))
        T_seqdict = {'base':T}
        
        if self.check_joint_limits(joint_angles):         
            for joint in self.robot.joint_keys:
                T = T*self.DH2T(DH[joint][0], DH[joint][1] + joint_angles[joint], 
                                DH[joint][2], DH[joint][3])
                T_seqdict[joint] = T
            
            if visualize:
                sx, sy, sz = self.convert2skel(T_seqdict)
                self.plot_skeleton(sx, sy, sz, proportion=False)
            
            if output == 't':            
                return np.around(T, 5)
            elif output == 'pr':
                return self.convert2pose(np.around(T, 5))
            else:
                raise ValueError("Invalid output form!")
        else:
            raise ValueError("Invalid FK Request!")

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

    def convert2pose(self, T):
        pose = {'x':T.item((0, 3)), 'y':T.item((1, 3)), 'z':T.item((2, 3))}
        R = T[0:3, 0:3]; euler = rotmat2euler(R)
        pose['rx'] = euler[0]
        pose['ry'] = euler[1]
        pose['rz'] = euler[2]
        return pose       
    
    def check_joint_limits(self, joint_angles):
        for joint in joint_angles.keys():
            lower_bound_check = joint_angles[joint] >= self.robot.angle_limits[joint][0]
            upper_bound_check = joint_angles[joint] <= self.robot.angle_limits[joint][1]
            if not (lower_bound_check and upper_bound_check):
                print "{} joint limits exceeded!".format(joint)
                return False
            
        return True
        
    def visualize_pose(self, joint_angles):
        self.solveFK(joint_angles, visualize=True)
    
    def convert2skel(self, T_seqdict):
        sx = [0]; sy = [0]; sz = [0]
        offset_dict = {'s0':'d1', 'e0':'d2', 'w0':'d3', 'w2':'d4'}
        
        for key in self.robot.joint_keys:
            T = T_seqdict[key]
            if key in ['s0', 'e0', 'w0', 'w2']:
                offset = self.robot.lengths[offset_dict[key]]
                sx += [T.item((0, 3)) - offset*T.item(0, 0)]
                sy += [T.item((1, 3)) - offset*T.item(1, 0)]
                sz += [T.item((2, 3)) - offset*T.item(2, 0)]
            else:
                sx += [T.item((0, 3))]
                sy += [T.item((1, 3))]
                sz += [T.item((2, 3))]
        
        return sx, sy, sz
                
    def plot_skeleton(self, xs, ys, zs, proportion=False):
        xs = np.around(xs, 5)
        ys = np.around(ys, 5)
        zs = np.around(zs, 5)
        
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.plot(xs, ys, zs, 'ro-')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        
        if proportion:
            max_val = max(max(abs(xs)), max(abs(ys)), max(abs(zs)))
            ax.set_xlim3d(-max_val, max_val)
            ax.set_ylim3d(-max_val, max_val)
            ax.set_zlim3d(-max_val, max_val)
        
        plt.show()