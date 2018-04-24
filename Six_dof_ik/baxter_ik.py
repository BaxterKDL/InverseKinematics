import numpy as np
from euts import *
import robot
import math
import cmath
import time
from FKSolver import FKSolver
from euts import *


baxter = robot.Baxter()
class IKSolver:
    def __init__(self, robot):
        self.robot = robot
        self.l = [0.27035, 0.069, 0.36435, 0.069, 0.37429, 0.01, 0.229525, 0.3708]
        self.d = [self.l[0], 0, 0, 0, self.l[4], 0, 0]
        self.a = [0, self.l[1], self.l[7], 0, 0, 0, 0]
        self.alpha = [0, -90, 0, 90, -90, 90, 0]
        self.theta = [0, 30 , 60 , 90 + 45, 60 , 90 , 30]

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

    def solveIK(self,pose):
        baxt = FKSolver(baxter)
        T = baxt.convert2T(pose)
    	# Taking the position values
        f_link = np.array([0,0,-self.l[6],1])
        f_link_ = np.dot(T,f_link.reshape(4,1))
        T[0:4,3] = f_link_.reshape(4,1)

    	x = T[0,3]
    	y = T[1,3]
    	z = T[2,3]
    	# Defining lengths
    	l0 = self.l[0]
    	l1 = self.l[1]
    	l2 = self.l[2]
    	l3 = self.l[3]
    	l4 = self.l[4]
    	l5 = self.l[5]
    	l6 = self.l[6]
    	lh = self.l[7]

    	z = z-l0

    	th1 = math.atan2(y,x)

    	# Calculating theta 2
    	E = np.around(2*lh*(l1-(x/math.cos(th1))), decimals=4)
    	F = np.around(2*lh*z, decimals=4);
    	G = np.around((((x**2)/((math.cos(th1))**2))) + (l1**2) + (lh**2) - (l4**2) + (z**2) - ((2*l1*x)/math.cos(th1)), decimals=4)
    	k =cmath.sqrt(E**2 + F**2 - G**2)

    	if k.imag != 0 :
    		print("Not Possible! ")
    		th = 0
    		return th

    	ta1 = (-F + k)/(G-E)
    	ta2 = (-F - k)/(G-E)

    	th21 = 2*math.atan(ta1)
    	th22 = 2*math.atan(ta2)

    	# Calculating theta 4

    	th41 = math.atan2(-z - lh*math.sin(th21),(x/math.cos(th1)) -l1 - lh*math.cos(th21)) - th21
    	th42 = math.atan2(-z - lh*math.sin(th22),(x/math.cos(th1)) -l1 - lh*math.cos(th22)) - th22

    	# For theta1 = th1
    	th_1 = [[0 for j in range(1)] for i in range(2)]
    	th_1[0][0]=th1
    	th_1[1][0]=th1

    	# For theta2 = th2
    	th_2 = [[0 for j in range(1)] for i in range(2)]
    	th_2[0][0]=th21
    	th_2[1][0]=th22

        # For theta3 = th3
        th_3 = [[0 for j in range(1)] for i in range(2)]
        th_3[0][0]=0
    	th_3[1][0]=0

    	# For theta4 =th4
    	th_4 = [[0 for j in range(1)] for i in range(2)]
    	th_4[0][0]=th41
    	th_4[1][0]=th42

    	# Finding R30s for th

        r = T[0:3,0:3]
        r_a = np.zeros((3,3,2),'float')
        th5 = [[0 for j in range(1)] for i in range(2)]
        th7 = [[0 for j in range(1)] for i in range(2)]
        th6 = [[0 for j in range(1)] for i in range(2)]
        sol_flag = [True for j in range(2)]
        joint_pose1 = {}
        joint_pose2 = {}

    	# For r301
    	T_new = np.identity(4)
    	d_th1 = rad2deg(th1)
    	d_th21 = rad2deg(th21)
    	d_th41 = rad2deg(th41)
    	th_temp1 = [0,d_th1,d_th21, 90 + d_th41]


        final_pose = {'s0':d_th1, 's1':d_th21-10.7236, 'e0':0, 'e1':d_th41+10.7236, 'w0':0, 'w1':0, 'w2':0}
        if baxt.check_joint_limits(final_pose):
            aa = baxt.solveIntermediateFK(final_pose)
            r301 = aa[3]
            r301 = r301[0:3,0:3]

            r1 = np.dot(np.matrix.transpose(r301),r)
            r_a[...,0] = r1
            q = math.atan2(r_a[1,2,0],r_a[0,2,0])
            th5[0][0]=q
            u = math.atan2(r_a[2,1,0],-r_a[2,0,0])
            th7[0][0]=u

            e0 = -r_a[2,0,0]/math.cos(u)
            f  = math.atan2(e0,r_a[2,2,0])

            th6[0][0]=f
            th_2[0][0]=th21 - 0.1872
            th_4[0][0]=th41 + 0.1872
            joint_pose1 = {}
            th111 =[th_1[0][0],th_2[0][0],th_3[0][0],th_4[0][0],th5[0][0],th6[0][0],th7[0][0]]
            for i in range(len(baxter.joint_keys)):
                joint_pose1[baxter.joint_keys[i]] =rad2deg(th111[i])
            if not baxt.check_joint_limits(joint_pose2):
                sol_flag[0] = False

        else:
            sol_flag[0] = False

    	T_rnew = np.identity(4)
    	d_th1 = rad2deg(th1)
    	d_th22 = rad2deg(th22)
    	d_th42 = rad2deg(th42)
    	th_temp2=[0,d_th1,d_th22, 90 + d_th42]

        final_pose = {'s0':d_th1, 's1':d_th22-10.7236, 'e0':0, 'e1':d_th42+10.7236, 'w0':0, 'w1':0, 'w2':0}
        if baxt.check_joint_limits(final_pose):
            aa = baxt.solveIntermediateFK(final_pose)
            r302 = aa[3]
            r302 = r302[0:3,0:3]

            r2 = np.dot(np.matrix.transpose(r302),r)
            r_a[...,1] = r2
        	# Finding theta 5
            p = math.atan2(r_a[1,2,1],r_a[0,2,1])
            th5[1][0]=p

        	# Finding theta 7
            v = math.atan2(r_a[2,1,1],-r_a[2,0,1])
            th7[1][0]=v
            e1 = -r_a[2,0,1]/math.cos(v)

            g  = math.atan2(e1,r_a[2,2,1])

            th6[1][0]=g

            th_2[1][0]=th22 - 0.1872
            th_4[1][0]=th42 + 0.1872

            joint_pose2 = {}
            th222 =[th_1[1][0],th_2[1][0],th_3[1][0],th_4[1][0],th5[1][0],th6[1][0],th7[1][0]]
            for i in range(len(baxter.joint_keys)):
                joint_pose2[baxter.joint_keys[i]] =rad2deg(th222[i])
            if not baxt.check_joint_limits(joint_pose2):
                sol_flag[1] = False

        else:
            sol_flag[1] = False

    	th =[th_1,th_2,th_3,th_4,th5,th6,th7]

        if sol_flag[0] == False and sol_flag[1]== False:
            # no Solution
            print "No IK solution!"
            return 0
        elif sol_flag[0] == False and sol_flag[1]== True:
            return joint_pose2
        elif sol_flag[0] == True and sol_flag[1]== False:
            return joint_pose1
        else:
            return baxt.compare_pose(joint_pose1,joint_pose2)


        # # joint_offset = {'s0':0, 's1':-0.1872, 'e0':0, 'e1':0.1872, 'w0':0, 'w1':0, 'w2':0}
        # joint_pose1 = {}
        # joint_pose2 = {}
        # for i in range(len(baxter.joint_keys)):
        #     joint_pose1[baxter.joint_keys[i]] =rad2deg(th[i][0][0])
        #     joint_pose2[baxter.joint_keys[i]] =rad2deg(th[i][1][0])
        #
        #
        #
    	# return joint_pose1





def main():
    # T = np.identity(4)
    # obj = IKSolver(baxter)
    # for i in range(0,7):
    #     T_1 = obj.DH2T(obj.d[i], obj.theta[i], obj.a[i], obj.alpha[i])
    #     T = T * T_1
    # print T
     obj = IKSolver(baxter)
     baxfk = FKSolver(baxter)
     #joint_pose = {'s0':30, 's1':20, 'e0':0, 'e1':30, 'w0':70, 'w1':60, 'w2':20}
     joint_pose = {'s0':-10, 's1':-5, 'e0':15, 'e1':25, 'w0':15, 'w1':10, 'w2':-15}
     print joint_pose
     T = baxfk.solveFK(joint_pose, visualize=False, output='t')
     print T
     # T =np.matrix([[1, 0, 0, pose['x']],
     #               [0, 1, 0, pose['y']],
     #               [0, 0, 1, pose['z']],
     #               [0, 0, 0, 1]])
     pose = {'rx':30, 'ry':20, 'rz':0, 'y':30, 'x':70, 'z':60}
     pose = baxfk.convert2pose(T)
     print pose

     sol =obj.solveIK(pose)
     print sol

     joint_pose2 = sol
     #print joint_pose
     T1 = baxfk.solveFK(joint_pose2, visualize=False, output='t')
     print T1


  #  T = baxfk.solveFK(sol, visualize=False, output='t')
   # print T
    #pose = baxfk.convert2pose(T)
    #print pose

# Main


if __name__ == "__main__":
  main()
