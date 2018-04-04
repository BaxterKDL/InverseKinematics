import numpy as np
from Quaternion import *

class Euler:
	
    def __init__(self, val = (0, 0, 0)):
        self.x = val[0]
        self.y = val[1]
        self.z = val[2]
    
    def e2q(self):
        quat = Quaternion((1, 0, 0, 0))
        
        chz = np.cos(self.z / 2.0)
        shz = np.sin(self.z / 2.0)
        chy = np.cos(self.y / 2.0)
        shy = np.sin(self.y / 2.0)
        chx = np.cos(self.x / 2.0)
        shx = np.sin(self.x / 2.0)
        
        quat.w = chz*chy*chx + shz*shy*shx
        quat.x = shz*chy*chx - chz*shy*shx
        quat.y = chz*shy*chx + shz*chy*shx
        quat.z = chz*chy*shx - shz*shy*chx
        
        return quat
    
    def value(self):
        return [self.x, self.y, self.z]
    
    def display(self):
        print self.value()