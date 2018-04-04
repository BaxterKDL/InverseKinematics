from FKSolver import *
import numpy  as np
import robot
from sympy import *


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

    def update_angle(self, flag, angle, keyvalue, step=0.5):

        '''Updating the angular position value, either positive or negative with the given step size '''

        # Checking the condition for positive or negative increament in joint angular position
        if flag == True:
            updatedAngle = angle + step
        else:
            updatedAngle = angle - step

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

    def check_ee_on_line_EXACT(self, p1, p2, cp):

        '''Checking if the EE lies on the line joining the rotating joint and the target position

        THIS IS ONE EXACT METHOD WHICH MIGHT NOT WORK GIVEN THAT WE IMPLEMENT DISCRETE INCREMENTS, SO THE POINT MIGHT BE
        REALLY CLOSE TO THE LINE AND MAY STILL NOT LIE ON IT EXACTLY

        '''

        # Using technique from this website - https://brilliant.org/wiki/3d-coordinate-geometry-equation-of-a-line/
        pos1, pos2, checkpoint = p1, p2, cp

        x, y, z = symbols('x,y,z')
        axes = [x, y, z]

        def create_line(pos1, pos2):

            '''Define the equation of the line on which the EE should eventually lie to shift the iteration to the next joint '''

            equationPart1, equationPart2 = [], []

            for i, a, b in [0, len(pos1)-1], pos1, pos2:
                dVectorCoeff = b-a
                if dVectorCoeff !=0:
                    equationPart1.append((axes[i] - a)/dVectorCoeff)
                else:
                    equationPart2.append(list((axes[i], a)))

            return equationPart1, equationPart2

        def check(pos1, pos2, checkpoint):

            '''Check if the EE lies on the line joining the target position and the joint being rotated'''

            eq1, eq2 = create_line(pos1, pos2)
            resOld1, resNew1, resOld2, resNew2 = 0, 0, 0, 0

            for indexA, component1 in [0, len(eq1)-1], eq1:
                resNew1 = component1.subs([(x,checkpoint[0]), (y, checkpoint[1]), (z, checkpoint[2])])
                if indexA != 0 and resNew1!=resOld1:
                    return False
                resOld1 = resNew1

            for indexB, component2 in [0, len(eq2)-1], eq2:
                resNew2 = component2[0].subs([(x,checkpoint[0]), (y, checkpoint[1]), (z, checkpoint[2])])
                if resNew2 != component2[1]:
                    return False

            return True

        flag = check(pos1, pos2, checkpoint)

        return flag

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
            return True
        else:
            # print ("Not done ye, trying to get EE on the joining line")
            return False

    def exec_ccd(self):

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

        while True:

            iter_num = -1
            maxout = 0

            # Need to ensure that the farthest joint is moved first and
            for keyvalue in reversed(ja_keys[0:5]):

                # print ("Iteration being done for joint:", keyvalue)
                iter_num +=1
                maxout = 0

                temp_d1 = self.cartesian_distance(self.ee_pos, self.target_position)

                while True:
                    counter +=1
                    # print ("Counter Value ", counter)

                    # Updating the angular position for the joint under consideration
                    updateResult = self.update_angle(flag, self.joint_anglesdict[keyvalue], keyvalue)


                    if updateResult !=1000:
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

                    # print ("Angle value for current joint", keyvalue, " is ", self.joint_anglesdict[keyvalue])

                    condition1 = self.check_ee_on_line_APPROX(self.joint_pos, self.target_position, self.ee_pos, 0.03)
                    # print ("Still working on the joint: ", keyvalue)

                    # print ("\nUpdated joint angles dictionary", joint_anglesdict, "\n\n")

                    if (condition1):
                        # print ("Condition 1 satisfied")
                        break

            self.ee_pos = self.angles_to_fkposition(self.joint_anglesdict)
            current_offset = self.cartesian_distance(self.target_position, self.ee_pos)

            print ("Distance between EE and target", current_offset)
            flag = not flag

            if current_offset < self.error_tolerance:
                break

            # if maxout == 7:
            #     print ("No solution found in the positive direction, so terminating the process")
            #     flag = not flag
            #     # break

        final_IK_solution = self.joint_anglesdict
        return final_IK_solution












