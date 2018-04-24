import random
import numpy as np
from euts import *
from FKSolver import FKSolver

class IKSolver:
    def __init__(self, robot, seed_jpose=None):
        self.robot = robot
        self.fk = FKSolver(robot)
        
        self.curr_jpose = None
        self.curr_tpose = None
        self.J = None
        if seed_jpose is not None: 
            self.set_current_pose(seed_jpose)
        
        self.vel_mag = 0.05
        self.min_dist = 0.01
        self.max_iters = 1000
        self.max_retries = 100
    
    def set_current_pose(self, jpose):
        self.curr_jpose = jpose
        self.curr_tpose = self.fk.solveFK(jpose, output='pr')
        self.J = self.robot.jacobian(jpose)
    
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
    
    def generate_random_jpose(self):
        jpose = {}
        for key in self.robot.joint_keys:
            alimit = self.robot.angle_limits[key]
            jpose[key] = random.uniform(alimit[0], alimit[1])
        return jpose
    
    def PsuedoInverse(self, end_tpose):
        
        iterations = 0
        retries = 0
        self.set_current_pose(self.generate_random_jpose())
        
        dist_check = dict_euclidean(self.curr_tpose, end_tpose) > self.min_dist
        iter_check = iterations < self.max_iters
        retry_check = retries < self.max_retries      
        
        while dist_check and retry_check:
            iterations += 1
            
            xdot_rad = self.generateTaskVel(self.curr_tpose, end_tpose)
            qdot_rad = self.J.pinv()*xdot_rad
            
            qdot_deg = rad2deg(qdot_rad)
            qdot_dict = vector2dict(qdot_deg, self.robot.joint_keys)
            new_curr_jpose = add_dicts(self.curr_jpose, qdot_dict)
            
            if not (self.fk.check_joint_limits(new_curr_jpose) and iter_check):
                retries += 1
                iterations = 0
                print "Retry", retries
                new_curr_jpose = self.generate_random_jpose()
            self.set_current_pose(new_curr_jpose)
            
            a = dict_truncate(self.curr_tpose, ["x", "y", "z"])
            b = dict_truncate(end_tpose, ["x", "y", "z"])
            
            dist_check = dict_euclidean(a, b) > self.min_dist
            iter_check = iterations < self.max_iters
            retry_check = retries < self.max_retries
        
        if not dist_check:
            print "Yay! Nailed it!"
        else:
            print "Sorry! I failed!"
        
        print "Retries = ", retries
        print "Iterations = ", iterations
        #print "Task Space Pose = ", self.curr_tpose
        print "Task Space Error = ", dict_euclidean(a, b)
        return self.curr_jpose
    
    def generateTaskVel(self, curr_tpose, end_tpose):
        xdot_pos = []
        xdot_rot = []
        for key in self.robot.task_pos_keys:
            xdot_pos += [[end_tpose[key] - curr_tpose[key]]]
        xdot_pos = self.vel_mag*(xdot_pos/np.linalg.norm(xdot_pos)) 
        for key in self.robot.task_rot_keys:
            xdot_rot += [[deg2rad(end_tpose[key] - curr_tpose[key])]]
        xdot_rot = self.vel_mag*(xdot_rot/np.linalg.norm(xdot_rot)) 
        
        xdot = np.matrix(np.append(xdot_pos, xdot_rot)).T
        return xdot
    
    def NullSpace(self, end_tpose):
        xdot_rad = np.matrix([[0]]*6)
        q_rand = np.matrix([[5]]*7)
        
        print self.J.matrix*self.J.nullprojector()
        qdot1 = self.J.pinv()*xdot_rad
        qdot2 = self.J.pinv()*xdot_rad + self.J.nullprojector()*q_rand
        
        qdot1_deg = rad2deg(qdot1)      
        qdot1_dict = vector2dict(qdot1_deg, self.robot.joint_keys)
        new_curr_jpose1 = add_dicts(self.curr_jpose, qdot1_dict)
        
        qdot2_deg = rad2deg(qdot2)
        qdot2_dict = vector2dict(qdot2_deg, self.robot.joint_keys)
        new_curr_jpose2 = add_dicts(self.curr_jpose, qdot2_dict)
        
        print qdot1, "\n"
        print self.J.matrix*qdot1, "\n"
        print self.fk.solveFK(new_curr_jpose1, output='pr'), "\n"
        print qdot2, "\n"
        print self.J.matrix*qdot2, "\n"
        print self.fk.solveFK(new_curr_jpose2, output='pr'), "\n"
        