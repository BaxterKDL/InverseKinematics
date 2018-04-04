import numpy as np

aa = [3,1,-1]
bb = [5, 2, 1]
cp = [0, 2, 3]

def check_ee_on_line_APPROX(p1, p2, cp, tol):

    '''Calculate the perpendicular distance between the EE and the line joining the rotating joint to the target point

    THIS IS THE IMPLEMENTATION OF APPROXIMATE SOLUTION TO DETERMINE IF THE POINT LIES ON THE LINE OR IN ITS VICINITY
    '''

    pos1, pos2, checkpoint, tolerance = p1, p2, cp, tol
    s_vector = []
    M_1 = pos1

    for index, (a, b) in enumerate(zip(pos1, pos2)):
        dVectorCoeff = b - a
        s_vector.append(dVectorCoeff)

    def distance_point_line_3D(pos1, pos2, checkPoint):

        ''' http://onlinemschool.com/math/library/analytic_geometry/p_line/

        '''
        dist =0

        M_0_M_1 = [p-q for (p, q) in zip(M_1, checkpoint) ]

        print M_0_M_1
        print s_vector
        cross_prod = np.cross(M_0_M_1, s_vector)
        dist = np.linalg.norm(cross_prod) / np.linalg.norm(s_vector)

        return dist

    distance = distance_point_line_3D(pos1, pos2, checkpoint)
    print distance
    if distance < tolerance:
        print "Done"
        return True
    else:
        print "Not done"
        return False

check_ee_on_line_APPROX(aa, bb, cp,2)

