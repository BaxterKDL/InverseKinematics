from FKSolver import FKSolver
from euts import *
import numpy as np

class IKSolver:
    
    def __init__(self, robot, cpose):
        self.robot = robot
        self.fk = FKSolver(robot)
        self.set_current_pose(cpose)
        self.vel_mag = 0.01
        self.min_dist = 1
    
    def set_current_pose(self, cpose):
        self.curr_jpose = cpose
        self.curr_tpose = self.fk.solveFK(cpose, output='pr')
        self.J = self.robot.jacobian(cpose)
    
    def lazy(self, j1, j2):
        jcurr = self.curr_jpose
        acc1 = 0; acc2 = 0
        
        for joint in self.robot.joint_keys:
            acc1 += pow(jcurr[joint]-j1[joint], 2)
            acc2 += pow(jcurr[joint]-j2[joint], 2)
        
        dist1 = np.sqrt(acc1)
        dist2 = np.sqrt(acc2)
        
        if dist1 < dist2:
            return j1, dist1
        else:
            return j2, dist2
    
    def PsuedoInverse(self, end_tpose):
        iterations = 0
        while dict_euclidean(self.curr_tpose, end_tpose) > self.min_dist:
            xdot_rad = self.generateTaskVel(self.curr_tpose, end_tpose)
            qdot_rad = self.J.pinv()*xdot_rad
            qdot_deg = rad2deg(qdot_rad)      
            qdot_dict = vector2dict(qdot_deg, self.robot.joint_keys)
            new_curr_jpose = add_dicts(self.curr_jpose, qdot_dict)
            self.set_current_pose(new_curr_jpose)
            iterations += 1
            
            print "Iteration", iterations
            print "Joint Pose ="
            print self.curr_jpose
            #print "Task Pose ="
            #print self.curr_tpose
            print "Task Space Error ="
            print dict_euclidean(self.curr_tpose, end_tpose)
            print "Yoshikawa ="
            print self.J.yoshikawa()
            
            #self.fk.visualize_pose(self.curr_jpose)
            
        #print "Iterations = ", iterations
        #print "Task Space Error = ", dict_euclidean(self.curr_tpose, end_tpose)
        return self.curr_jpose
    
    def generateTaskVel(self, curr_tpose, end_tpose):
        xdot = []
        for key in self.robot.task_pos_keys:
            xdot += [[end_tpose[key] - curr_tpose[key]]]
        for key in self.robot.task_rot_keys:
            #xdot += [[deg2rad(end_tpose[key] - curr_tpose[key])]]
            xdot += [[0]]
        xdot = np.matrix(xdot)
        xdot = xdot/np.linalg.norm(xdot)

        return self.vel_mag*xdot