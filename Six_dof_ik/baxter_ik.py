import numpy as np
from mathfunctions import *
import robot
import math


baxter = robot.Baxter()
class IKSolver:
    def __init__(self, robot):
        self.robot = robot
        self.l = [0.27035, 0.069, 0.36435, 0.069, 0.37429, 0.01, 0.36830, 0.3708]
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
    def solveIK(self,T):

		# Taking the position values
		x = T[0,3]
		y = T[1,3]
		z = T[2,3]
		print 'T = ',T
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
		E = 2*lh*(l1-(x/math.cos(th1)))
		F = 2*lh*z;
		G = (((x**2)/((math.cos(th1))**2))) + (l1**2) + (lh**2) - (l4**2) + (z**2) - ((2*l1*x)/math.cos(th1))
		k =math.sqrt(E**2 + F**2 - G**2)

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

		# For theta4 =th4 
		th_4 = [[0 for j in range(1)] for i in range(2)]
		th_4[0][0]=th41
		th_4[1][0]=th42
		
		# Finding R30s for th
		

		# For r301
		T_new = np.identity(4)
		d_th1 = rad2deg(th1)
		d_th21 = rad2deg(th21)
		d_th41 = rad2deg(th41)
		th_temp1 = [0,d_th1,d_th21, 90 + d_th41]
		
		for i in range(0,4):      
			r30 = self.DH2T(self.d[i], th_temp1[i], self.a[i], self.alpha[i])	
			T_new = T_new * r30 
		r301= T_new[0:3,0:3]
				
		# For r302
		T_rnew = np.identity(4)
		d_th1 = rad2deg(th1)
		d_th22 = rad2deg(th22)
		d_th42 = rad2deg(th42)
		th_temp2=[0,d_th1,d_th22, 90 + d_th42]

		for j in range(0,4):      
			Tr30 = self.DH2T(self.d[j], th_temp2[j], self.a[j], self.alpha[j])
			T_rnew = T_rnew * Tr30 
		r302= T_rnew[0:3,0:3]

		# Concatenating matrices
		r = T[0:3,0:3]
		r1 = np.matrix.transpose(r301)*r
		r2 = np.matrix.transpose(r302)*r
		r_a = np.zeros((3,3,2),'float')
		r_a[...,0] = r1
		r_a[...,1] = r2
		
		# Finding theta 5
		q = math.atan2(r_a[1,2,0],r_a[0,2,0])
		p = math.atan2(r_a[1,2,1],r_a[0,2,1])
		th5 = [[0 for j in range(1)] for i in range(2)]
		th5[0][0]=q
		th5[1][0]=p

		# Finding theta 7
		u = math.atan2(r_a[2,1,0],-r_a[2,0,0])
		v = math.atan2(r_a[2,1,1],-r_a[2,0,1])
		th7 = [[0 for j in range(1)] for i in range(2)]
		th7[0][0]=u
		th7[1][0]=v

		# Finding theta 6

		e0 = -r_a[2,0,0]/math.cos(u)
		e1 = -r_a[2,0,1]/math.cos(v)
		f  = math.atan2(e0,r_a[2,2,0])
		g  = math.atan2(e1,r_a[2,2,1])
		th6 = [[0 for j in range(1)] for i in range(2)]
		th6[0][0]=f
		th6[1][0]=g

		th =[th_1,th_2,th_4,th5,th6,th7]

		return th




      
def main():
	T = np.identity(4)
	obj = IKSolver(baxter)
	for i in range(0,7):      
		T_1 = obj.DH2T(obj.d[i], obj.theta[i], obj.a[i], obj.alpha[i])
		T = T * T_1 
			
	sol =obj.solveIK(T)
	print sol

# Main 
  

if __name__ == "__main__":
  main()


  


