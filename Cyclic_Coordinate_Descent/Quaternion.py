import numpy as np
from Euler import Euler

class Quaternion:
    def __init__(self, val = (1, 0, 0, 0)):
        self.w = val[0]
        self.x = val[1]
        self.y = val[2]
        self.z = val[3]
        self.normalize()
    
    def normalize(self):
        norm = pow(pow(self.w, 2)+pow(self.x, 2)+pow(self.y, 2)+pow(self.z, 2), 0.5)
        self.w = self.w/norm
        self.x = self.x/norm
        self.y = self.y/norm
        self.z = self.z/norm
        
    def q2e(self):
        euler = Euler((0, 0, 0))
        
        sinx = 2.0*(self.w*self.x + self.y*self.z)
        cosx = 1.0 - 2.0*(self.x*self.x + self.y*self.y)
        euler.x = np.arctan2(sinx, cosx)
        
        siny = 2.0*(self.w*self.y - self.z*self.x)
        euler.y = np.arcsin(siny)
        
        sinz = 2.0*(self.w*self.z + self.x*self.y)
        cosz = 1.0 - 2.0*(self.y*self.y + self.z*self.z)
        euler.z = np.arctan2(sinz, cosz)
        
        return euler
        
    def value(self):
        return [self.w, self.x, self.y, self.z]
    
    def display(self):
        print self.value()