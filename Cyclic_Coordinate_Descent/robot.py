import baxter_jacobian as baxJ
from Jacobian import Jacobian
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from euts import *
from FKSolver import *

class Baxter:

    def __init__(self):
        self.lengths = {'l1': 0.27035, 'd1': 0.069, 'l2': 0.36435, 'd2': 0.069,
                        'l3': 0.37429, 'd3': 0.01, 'l4': 0.229525, 'd4': 0}
        self.link_masses = {'m1':0.5, 'm2':0.5, 'm3':0.5, 'm4':0.5}

        self.angle_limits = {'s0': [-141, 51], 's1': [-123, 60],
                            'e0': [-173.5, 173.5], 'e1': [-3, 150],
                            'w0': [-175.25, 175.25], 'w1': [-90, 120],
                            'w2': [-175.25, 175.25]}

        self.joint_keys = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
        self.task_pos_keys = ['x', 'y', 'z']
        self.task_rot_keys = ['rx', 'ry', 'rz']
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

    def jacobian(self, joint_angles):
        j_matrix = baxJ.jacobian_matrix(joint_angles, self.lengths)
        J = Jacobian(j_matrix)
        return J

    def workspace(self, robot, num_samples):

        robotfk = FKSolver(robot)

        s0_limits = robot.angle_limits['s0']
        s1_limits = robot.angle_limits['s1']
        e0_limits = robot.angle_limits['e0']
        e1_limits = robot.angle_limits['e1']
        w0_limits = robot.angle_limits['w0']
        w1_limits = robot.angle_limits['w1']
        w2_limits = robot.angle_limits['w2']

        s0_range = np.linspace(s0_limits[0], s0_limits[1], num_samples)
        s1_range = np.linspace(s1_limits[0], s1_limits[1], num_samples)
        e0_range = np.linspace(e0_limits[0], e0_limits[1], num_samples)
        e1_range = np.linspace(e1_limits[0], e1_limits[1], num_samples)
        w0_range = np.linspace(w0_limits[0], w0_limits[1], num_samples)
        w1_range = np.linspace(w1_limits[0], w1_limits[1], num_samples)
        w2_range = np.linspace(w2_limits[0], w2_limits[1], num_samples)

        ws_x = []; ws_y = []; ws_z = []
        posesAnalysed = 0

        for s0 in s0_range:
            for s1 in s1_range:
                for e0 in e0_range:
                    for e1 in e1_range:
                        for w0 in w0_range:
                            for w1 in w1_range:
                                joint_pose = {'s0':s0, 's1':s1, 'e0':e0, 'e1':e1,
                                                'w0':w0, 'w1':w1, 'w2':0}
                                T = robotfk.solveFK(joint_pose)
                                ws_x += [T.item((0, 3))]
                                ws_y += [T.item((1, 3))]
                                ws_z += [T.item((2, 3))]
                                posesAnalysed += 1

        print "Poses Analysed = {}".format(posesAnalysed)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(ws_x, ws_y, ws_z)
        ax.scatter(0, 0, 0, c='r', marker='o')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        plt.show()

#TODO : Make generalized robot class for other robots