# Created on Fri Feb 16 20:13:27 2018
# Authors: Chaitanya Perugu, Ashwin Sahasrabudhe

#%%
import numpy as np
from Quaternion import Quaternion
import robot
from FKSolver import FKSolver
from IKSolver import IKSolver

baxter = robot.Baxter()

#%% Forward Pose Kinematics
baxfk = FKSolver(baxter)

print "Pose 1: End Pose"
joint_pose = {'s0':-130, 's1':-112, 'e0':148.5, 'e1':150, 'w0':175.25, 'w1':120, 'w2':175.25}
pose = baxfk.solveFK(joint_pose, visualize=True, output='pr')
print pose

print "Pose 2: Visualization"
joint_pose = {'s0':-45, 's1':30, 'e0':-30, 'e1':30, 'w0':-45, 'w1':0, 'w2':100}
#baxfk.visualize_pose(joint_pose)

#%% Quaternion Euler Conversion
quat = Quaternion((0, 1, 0, 0))
euler = quat.q2e()

print "Quaternion = {}".format(quat.value())
print "Euler angles = "
euler.display()

#%% Jacobians
#joint_pose = {'s0':0, 's1':0, 'e0':0, 'e1':0, 'w0':0, 'w1':0, 'w2':0}
joint_pose = {'s1': 59.5, 's0': 51, 'w2': 175.25, 'w1': -90, 'w0': -175.25, 'e1':-3, 'e0': 103}

print ("------------------------------------------------------------")
print ("------------------------------------------------------------")
print (baxfk.solveFK(joint_pose))

J = baxter.jacobian(joint_pose)
print "Jacobian = "
J.display()

print "Is Jacobian singular? ", J.is_singular()
print "Pseudoinverse = "
print np.around(J.pinv(), 3)
print "Null projector = "
print np.around(J.nullprojector(), 3)
print "Yoshikawa Manipulability Index = "
print J.yoshikawa()

#%% Workspace
baxter.workspace(baxter, 3)

#%% Inverse Pose Kinematics
curr_jpose =  {'s0':15, 's1':10, 'e0':-10, 'e1':10, 'w0':-15, 'w1':0, 'w2':10}
baxik = IKSolver(baxter, curr_jpose)
curr_tpose = baxik.fk.solveFK(curr_jpose, output='pr')

j1 = {'s0':0, 's1':10, 'e0':0, 'e1':45, 'w0':0, 'w1':0, 'w2':0}
j2 = {'s0':0, 's1':0, 'e0':15, 'e1':0, 'w0':20, 'w1':10, 'w2':0}
end_tpose = {'x':0.25, 'y':-0.25, 'z':0.25, 'rx':0, 'ry':0, 'rz':0}

print curr_tpose
ikans = baxik.PsuedoInverse(end_tpose)
print ikans
print baxik.lazy(j1, j2)
