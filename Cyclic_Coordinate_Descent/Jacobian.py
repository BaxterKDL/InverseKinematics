import numpy as np

class Jacobian:
    
    def __init__(self, j_matrix):
        self.matrix = j_matrix
        self.min_det_val = 1e-8
    
    def is_singular(self):
        J = self.matrix
        if np.linalg.det(J*J.T) < self.min_det_val:
            return True
        else:
            return False
    
    def pinv(self):
        if self.is_singular():
            raise ValueError("J is singular. Don't compute pinv!")
        else:
            J = self.matrix
            return J.T*(J*J.T).I
    
    def nullprojector(self):
        if self.is_singular():
            raise ValueError("J is singular. Don't compute nullprojector!")
        else:
            leftmult = self.pinv()*self.matrix
            I = np.eye(leftmult.shape[0], leftmult.shape[1])
            return (I - leftmult)
    
    def yoshikawa(self):
        if self.is_singular():
            print "J is singular. Don't compute Yoshikawa!"
            return 0
        else:
            J = self.matrix
            return np.sqrt(np.linalg.det(J*J.T))
        
    def display(self):
        print np.around(self.matrix, 3)