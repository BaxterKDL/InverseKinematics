from FKSolver import *
import numpy  as np
import robot
import matplotlib.pyplot as plt


class CycCorDes:

    ''' Cyclic Coordinate Descent where each link if moved sequentially, starting from one farthest from the base frame
        and the end-effector alignment with the target position is attempted
    '''

    #Creating the objects for the robot and the FK solver from the FKSolver library

    def __init__(self, current_seed_angles_dict, target_position, error_tolerance):

        '''Defining the class variables to be used multiple times'''

        self.seed_angles_dict = current_seed_angles_dict
        self.target_position = target_position
        self.error_tolerance = error_tolerance
        self.baxter = robot.Baxter()
        self.baxfk = FKSolver(self.baxter)
        self.joint_pos = None
        self.joint_pp = None
        self.ee_pos = [0,0,0]
        self.joint_anglesdict  = {}
        # self.figure = plt.figure()

        self.reversedJointList = []

        self.minStepSize = 0.2
        self.maxStepSize = 2
        self.stepSize = self.maxStepSize

    def angles_to_fkposition(self, currentJointAnglesDict):

        '''Computing the cartesian coordinate position of the end-effector given the joint angles '''

        T = self.baxfk.solveFK(currentJointAnglesDict)
        return T[0:3, 3]

    def cartesian_distance(self, pos1, pos2):

        '''Caculate the distance between two points in 2D/3D'''

        dis_sqr = 0

        for p1, p2 in zip(pos1, pos2):
            dis_sqr += (p1-p2)**2
        return np.sqrt(dis_sqr)

    def update_angle(self, flag, angle, keyvalue):

        '''Updating the angular position value, either positive or negative with the given step size '''

        # Checking the condition for positive or negative increament in joint angular position
        if flag == True:
            updatedAngle = angle + self.stepSize
        else:
            updatedAngle = angle - self.stepSize

        # if updatedAngle <= self.baxter.angle_limits[keyvalue][0] :
        #     return self.baxter.angle_limits[keyvalue][0], 1
        # elif updatedAngle >= self.baxter.angle_limits[keyvalue][1]:
        #     return self.baxter.angle_limits[keyvalue][1], 1
        # else:
        #     return updatedAngle, 0


        if self.baxter.angle_limits[keyvalue][0] <= updatedAngle <= self.baxter.angle_limits[keyvalue][1]:
            return updatedAngle
        else:
            return 1000


    def check_ee_on_line_APPROX(self, p1, p2, cp, tol):

        '''Calculate the perpendicular distance between the EE and the line joining the rotating joint to the target point
        THIS IS THE IMPLEMENTATION OF APPROXIMATE SOLUTION TO DETERMINE IF THE POINT LIES ON THE LINE OR IN ITS VICINITY
        '''

        pos1, pos2, checkpoint, tolerance = p1, p2, cp, tol
        s_vector = []
        M_1 = pos1

        for  a, b in zip(pos1, pos2):
            dVectorCoeff = b - a
            s_vector.append(dVectorCoeff)

        def distance_point_line_3D():

            ''' http://onlinemschool.com/math/library/analytic_geometry/p_line/

            '''
            # print ("M1 is: ",  M_1,  "\n\n")
            # print ("Checkpoint is: ", checkpoint, "\n\n")

            M_0_M_1 = [p - q for (p, q) in zip(M_1, checkpoint)]

            # print ("M_o_M_1  vector", M_0_M_1)
            # print ("S Vector", s_vector)

            cross_prod = np.cross(M_0_M_1, s_vector)
            dist = np.linalg.norm(cross_prod) / np.linalg.norm(s_vector)

            return dist

        distance = distance_point_line_3D()
        # print ("Distance between EE and joining line: ", distance, "\n")

        if distance < tolerance:
            # print ("Done - EE lies approximately on the line")
            return True, distance
        else:
            # print ("Not done ye, trying to get EE on the joining line")
            return False, distance

    def show_animation(self, complete_T, figu ):
        '''Extract the xs, ys and the zs from all the individual transformation matrices'''
        T_dict = {}
        ja_keys = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
        for t_mat, key in zip(complete_T, ja_keys):
            T_dict[key] = t_mat

        xs_a, ys_a, zs_a = self.baxfk.convert2skel(T_dict)
        self.baxfk.plot_skeleton(xs_a, ys_a, zs_a, figu, True)

    def exec_ccd(self, animate =False):

        '''Implementing the cyclic Coordinate Descent algorithm'''

        ja_keys = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']

        for ii in ja_keys:
            self.joint_anglesdict[ii] = self.seed_angles_dict[ii]
        # print ("Joint Dictionary", joint_anglesdict)

        current_offset = np.inf
        self.ee_pos = self.angles_to_fkposition(self.joint_anglesdict)

        counter  = -1
        flag = True
        temp_d1 = np.inf

        self.reversedJointList = list(ja_keys[0:6])
        self.reversedJointList.reverse()

        figu = plt.figure()
        ax = figu.gca(projection='3d')

        # self.joint_pp = list(self.baxfk.solveIntermediateFK(self.joint_anglesdict))

        while True:

            iter_num = -1
            maxout = 0

            self.joint_pp = list(self.baxfk.solveIntermediateFK(self.joint_anglesdict))

            # Need to ensure that the farthest joint is moved first
            for jointIterator in xrange(6):

            	keyvalue = self.reversedJointList[jointIterator]

                #print ("Iteration being done for joint:", keyvalue)
                iter_num +=1
                maxout = 0

                temp_d1 = self.cartesian_distance(self.ee_pos, self.target_position)

                self.stepSize = self.maxStepSize

                while True:

                    # print ("Angle value for current joint", keyvalue, " is ", self.joint_anglesdict[keyvalue])
                    # print ("Angle dictionary :", self.joint_anglesdict)

                    # print ("Still working on the joint: ", keyvalue)
                    counter +=1
                    # print ("Counter Value ", counter)

                    # Updating the angular position for the joint under consideration
                    updateResult = self.update_angle(flag, self.joint_anglesdict[keyvalue], keyvalue)


                    if updateResult!=1000:
                        self.joint_anglesdict[keyvalue]= round(updateResult, 2)
                    else:
                        # print("Angle maxed out")
                        maxout +=1
                        break

                    temp_d2 = self.cartesian_distance(self.angles_to_fkposition(self.joint_anglesdict), self.target_position)

                    if temp_d2>temp_d1:
                        # print (temp_d1, temp_d2)
                        flag = not flag
                        break

                    temp_d1 = temp_d2

                    self.joint_pp = list(self.baxfk.solveIntermediateFK(self.joint_anglesdict))
                    # print ("Joint Position - Compilation of intermediate matrices")

                    self.joint_pos = self.joint_pp[iter_num][0:3,3]

                    # print (joint_pp)
                    # print ("  ")
                    # print (joint_pos)
                    # print (joint_pos)

                    self.ee_pos = self.angles_to_fkposition(self.joint_anglesdict)

                    ee_OnLineCondition, distanceFromLine  = self.check_ee_on_line_APPROX(self.joint_pos, self.target_position, self.ee_pos, 0.02)

                    if distanceFromLine<0.05:
                    	self.stepSize = self.minStepSize

                    # print "Distance from line:",distanceFromLine

                    # print ("\nUpdated joint angles dictionary", joint_anglesdict, "\n\n")

                    if (ee_OnLineCondition):
                        # print ("Condition 1 satisfied")
                        break

                    self.ee_pos = self.angles_to_fkposition(self.joint_anglesdict)
                    current_offset = self.cartesian_distance(self.target_position, self.ee_pos)
                    #print ("Distance between EE and target", current_offset)

                    # print self.stepSize

                    if animate:
                        ax.scatter(self.target_position[0],  self.target_position[1], self.target_position[2], 'g')
                        self.show_animation(self.joint_pp, figu)
                        plt.cla()

            flag = not flag

            # if current_offset < 0.1:
            # 	jointIterator = 0
            # 	self.stepSize = 0.1
            # 	print "Joint iterator                                                goes back to link 1"
            # 	return

            if current_offset < self.error_tolerance:
                break

            # if maxout == 7:
            #     print ("No solution found in the positive direction, so terminating the process")
            #     flag = not flag
            #     # break
        if animate:
            plt.show(block=False)

        final_IK_solution = self.joint_anglesdict
        return final_IK_solution