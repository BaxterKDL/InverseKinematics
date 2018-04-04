# Created on Sat Mar 03 00:43:21 2018
# Author: Chaitanya Pb

import numpy as np

def jacobian_matrix(joint_angles, lengths):
    l1 = lengths['l1']
    d1 = lengths['d1']
    l2 = lengths['l2']
    d2 = lengths['d2']
    l3 = lengths['l3']
    d3 = lengths['d3']
    l4 = lengths['l4']
    d4 = lengths['d4']
    
    q0 = joint_angles['s0']
    q1 = joint_angles['s1']
    q2 = joint_angles['e0']
    q3 = joint_angles['e1']
    q4 = joint_angles['w0']
    q5 = joint_angles['w1']
    q6 = joint_angles['w2']    
    
    pi = np.pi
    
    A1 = [d3*np.cos(q4)*(np.cos(q1)*np.sin(q0)*np.sin(q3) - np.cos(q0)*np.cos(q3)*np.sin(q2) + 
         np.cos(q2)*np.cos(q3)*np.sin(q0)*np.sin(q1)) - d1*np.sin(q0) - 
         l3*(np.cos(q1)*np.cos(q3)*np.sin(q0) + np.cos(q0)*np.sin(q2)*np.sin(q3) - 
         np.cos(q2)*np.sin(q0)*np.sin(q1)*np.sin(q3)) - d3*np.sin(q4)*(np.cos(q0)*np.cos(q2) + 
         np.sin(q0)*np.sin(q1)*np.sin(q2)) - d2*np.cos(q0)*np.sin(q2) - l2*np.cos(q1)*np.sin(q0) - 
         l4*(np.cos(q5)*(np.cos(q1)*np.cos(q3)*np.sin(q0) + np.cos(q0)*np.sin(q2)*np.sin(q3) - 
         np.cos(q2)*np.sin(q0)*np.sin(q1)*np.sin(q3)) - 
         np.sin(q5)*(np.cos(q4)*(np.cos(q1)*np.sin(q0)*np.sin(q3) - np.cos(q0)*np.cos(q3)*np.sin(q2) + 
         np.cos(q2)*np.cos(q3)*np.sin(q0)*np.sin(q1)) - np.sin(q4)*(np.cos(q0)*np.cos(q2) + 
         np.sin(q0)*np.sin(q1)*np.sin(q2)))) + d2*np.cos(q2)*np.sin(q0)*np.sin(q1), 

         d3*np.cos(q4)*(np.cos(q0)*np.sin(q1)*np.sin(q3) - np.cos(q0)*np.cos(q1)*np.cos(q2)*np.cos(q3)) - 
         l3*(np.cos(q0)*np.cos(q3)*np.sin(q1) + np.cos(q0)*np.cos(q1)*np.cos(q2)*np.sin(q3)) - 
         l2*np.cos(q0)*np.sin(q1) - l4*(np.cos(q5)*(np.cos(q0)*np.cos(q3)*np.sin(q1) + 
         np.cos(q0)*np.cos(q1)*np.cos(q2)*np.sin(q3)) - 
         np.sin(q5)*(np.cos(q4)*(np.cos(q0)*np.sin(q1)*np.sin(q3) - 
         np.cos(q0)*np.cos(q1)*np.cos(q2)*np.cos(q3)) + np.cos(q0)*np.cos(q1)*np.sin(q2)*np.sin(q4))) - 
         d2*np.cos(q0)*np.cos(q1)*np.cos(q2) + d3*np.cos(q0)*np.cos(q1)*np.sin(q2)*np.sin(q4), 

         d3*np.sin(q4)*(np.sin(q0)*np.sin(q2) + np.cos(q0)*np.cos(q2)*np.sin(q1)) - 
         l4*(np.cos(q5)*(np.cos(q2)*np.sin(q0)*np.sin(q3) - np.cos(q0)*np.sin(q1)*np.sin(q2)*np.sin(q3)) + 
         np.sin(q5)*(np.cos(q4)*(np.cos(q2)*np.cos(q3)*np.sin(q0) - 
         np.cos(q0)*np.cos(q3)*np.sin(q1)*np.sin(q2)) - np.sin(q4)*(np.sin(q0)*np.sin(q2) + 
         np.cos(q0)*np.cos(q2)*np.sin(q1)))) - l3*(np.cos(q2)*np.sin(q0)*np.sin(q3) - 
         np.cos(q0)*np.sin(q1)*np.sin(q2)*np.sin(q3)) - d2*np.cos(q2)*np.sin(q0) - 
         d3*np.cos(q4)*(np.cos(q2)*np.cos(q3)*np.sin(q0) - np.cos(q0)*np.cos(q3)*np.sin(q1)*np.sin(q2)) + 
         d2*np.cos(q0)*np.sin(q1)*np.sin(q2),   

         d3*np.cos(q4)*(np.sin(q0)*np.sin(q2)*np.sin(q3) - np.cos(q0)*np.cos(q1)*np.cos(q3) + 
         np.cos(q0)*np.cos(q2)*np.sin(q1)*np.sin(q3)) - l3*(np.cos(q0)*np.cos(q1)*np.sin(q3) + 
         np.cos(q3)*np.sin(q0)*np.sin(q2) + np.cos(q0)*np.cos(q2)*np.cos(q3)*np.sin(q1)) - 
         l4*(np.cos(q5)*(np.cos(q0)*np.cos(q1)*np.sin(q3) + np.cos(q3)*np.sin(q0)*np.sin(q2) + 
         np.cos(q0)*np.cos(q2)*np.cos(q3)*np.sin(q1)) - 
         np.cos(q4)*np.sin(q5)*(np.sin(q0)*np.sin(q2)*np.sin(q3) - np.cos(q0)*np.cos(q1)*np.cos(q3) + 
         np.cos(q0)*np.cos(q2)*np.sin(q1)*np.sin(q3))), 

         l4*np.sin(q5)*(np.sin(q4)*(np.cos(q0)*np.cos(q1)*np.sin(q3) + np.cos(q3)*np.sin(q0)*np.sin(q2) + 
         np.cos(q0)*np.cos(q2)*np.cos(q3)*np.sin(q1)) - np.cos(q4)*(np.cos(q2)*np.sin(q0) - 
         np.cos(q0)*np.sin(q1)*np.sin(q2))) - d3*np.cos(q4)*(np.cos(q2)*np.sin(q0) - 
         np.cos(q0)*np.sin(q1)*np.sin(q2)) + d3*np.sin(q4)*(np.cos(q0)*np.cos(q1)*np.sin(q3) + 
         np.cos(q3)*np.sin(q0)*np.sin(q2) + np.cos(q0)*np.cos(q2)*np.cos(q3)*np.sin(q1)), 

         l4*(np.sin(q5)*(np.sin(q0)*np.sin(q2)*np.sin(q3) - np.cos(q0)*np.cos(q1)*np.cos(q3) + 
         np.cos(q0)*np.cos(q2)*np.sin(q1)*np.sin(q3)) - 
         np.cos(q5)*(np.cos(q4)*(np.cos(q0)*np.cos(q1)*np.sin(q3) + np.cos(q3)*np.sin(q0)*np.sin(q2) + 
         np.cos(q0)*np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q4)*(np.cos(q2)*np.sin(q0) - 
         np.cos(q0)*np.sin(q1)*np.sin(q2)))), 

         0]
    
    A2 = [d1*np.cos(q0) - l4*(np.cos(q5)*(np.sin(q0)*np.sin(q2)*np.sin(q3) - 
         np.cos(q0)*np.cos(q1)*np.cos(q3) + np.cos(q0)*np.cos(q2)*np.sin(q1)*np.sin(q3)) + 
         np.sin(q5)*(np.cos(q4)*(np.cos(q0)*np.cos(q1)*np.sin(q3) + np.cos(q3)*np.sin(q0)*np.sin(q2) + 
         np.cos(q0)*np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q4)*(np.cos(q2)*np.sin(q0) - 
         np.cos(q0)*np.sin(q1)*np.sin(q2)))) - l3*(np.sin(q0)*np.sin(q2)*np.sin(q3) - 
         np.cos(q0)*np.cos(q1)*np.cos(q3) + np.cos(q0)*np.cos(q2)*np.sin(q1)*np.sin(q3)) - 
         d3*np.sin(q4)*(np.cos(q2)*np.sin(q0) - np.cos(q0)*np.sin(q1)*np.sin(q2)) + 
         l2*np.cos(q0)*np.cos(q1) - d2*np.sin(q0)*np.sin(q2) - 
         d3*np.cos(q4)*(np.cos(q0)*np.cos(q1)*np.sin(q3) + np.cos(q3)*np.sin(q0)*np.sin(q2) + 
         np.cos(q0)*np.cos(q2)*np.cos(q3)*np.sin(q1)) - d2*np.cos(q0)*np.cos(q2)*np.sin(q1), 

         d3*np.cos(q4)*(np.sin(q0)*np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)*np.sin(q0)) - 
         l3*(np.cos(q3)*np.sin(q0)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q0)*np.sin(q3)) - 
         l2*np.sin(q0)*np.sin(q1) - l4*(np.cos(q5)*(np.cos(q3)*np.sin(q0)*np.sin(q1) + 
         np.cos(q1)*np.cos(q2)*np.sin(q0)*np.sin(q3)) - 
         np.sin(q5)*(np.cos(q4)*(np.sin(q0)*np.sin(q1)*np.sin(q3) - 
         np.cos(q1)*np.cos(q2)*np.cos(q3)*np.sin(q0)) + np.cos(q1)*np.sin(q0)*np.sin(q2)*np.sin(q4))) - 
         d2*np.cos(q1)*np.cos(q2)*np.sin(q0) + d3*np.cos(q1)*np.sin(q0)*np.sin(q2)*np.sin(q4), 

         l3*(np.cos(q0)*np.cos(q2)*np.sin(q3) + np.sin(q0)*np.sin(q1)*np.sin(q2)*np.sin(q3)) + 
         l4*(np.cos(q5)*(np.cos(q0)*np.cos(q2)*np.sin(q3) + np.sin(q0)*np.sin(q1)*np.sin(q2)*np.sin(q3)) + 
         np.sin(q5)*(np.cos(q4)*(np.cos(q0)*np.cos(q2)*np.cos(q3) + 
         np.cos(q3)*np.sin(q0)*np.sin(q1)*np.sin(q2)) - np.sin(q4)*(np.cos(q0)*np.sin(q2) - 
         np.cos(q2)*np.sin(q0)*np.sin(q1)))) - d3*np.sin(q4)*(np.cos(q0)*np.sin(q2) - 
         np.cos(q2)*np.sin(q0)*np.sin(q1)) + d2*np.cos(q0)*np.cos(q2) + 
         d3*np.cos(q4)*(np.cos(q0)*np.cos(q2)*np.cos(q3) + np.cos(q3)*np.sin(q0)*np.sin(q1)*np.sin(q2)) + 
         d2*np.sin(q0)*np.sin(q1)*np.sin(q2), 

         -l4*(np.cos(q5)*(np.cos(q1)*np.sin(q0)*np.sin(q3) - np.cos(q0)*np.cos(q3)*np.sin(q2) + 
         np.cos(q2)*np.cos(q3)*np.sin(q0)*np.sin(q1)) + 
         np.cos(q4)*np.sin(q5)*(np.cos(q1)*np.cos(q3)*np.sin(q0) + np.cos(q0)*np.sin(q2)*np.sin(q3) - 
         np.cos(q2)*np.sin(q0)*np.sin(q1)*np.sin(q3))) - l3*(np.cos(q1)*np.sin(q0)*np.sin(q3) - 
         np.cos(q0)*np.cos(q3)*np.sin(q2) + np.cos(q2)*np.cos(q3)*np.sin(q0)*np.sin(q1)) - 
         d3*np.cos(q4)*(np.cos(q1)*np.cos(q3)*np.sin(q0) + np.cos(q0)*np.sin(q2)*np.sin(q3) - 
         np.cos(q2)*np.sin(q0)*np.sin(q1)*np.sin(q3)), 

         d3*np.cos(q4)*(np.cos(q0)*np.cos(q2) + np.sin(q0)*np.sin(q1)*np.sin(q2)) + 
         l4*np.sin(q5)*(np.sin(q4)*(np.cos(q1)*np.sin(q0)*np.sin(q3) - np.cos(q0)*np.cos(q3)*np.sin(q2) + 
         np.cos(q2)*np.cos(q3)*np.sin(q0)*np.sin(q1)) + np.cos(q4)*(np.cos(q0)*np.cos(q2) + 
         np.sin(q0)*np.sin(q1)*np.sin(q2))) + d3*np.sin(q4)*(np.cos(q1)*np.sin(q0)*np.sin(q3) - 
         np.cos(q0)*np.cos(q3)*np.sin(q2) + np.cos(q2)*np.cos(q3)*np.sin(q0)*np.sin(q1)), 

         -l4*(np.sin(q5)*(np.cos(q1)*np.cos(q3)*np.sin(q0) + np.cos(q0)*np.sin(q2)*np.sin(q3) - 
         np.cos(q2)*np.sin(q0)*np.sin(q1)*np.sin(q3)) + 
         np.cos(q5)*(np.cos(q4)*(np.cos(q1)*np.sin(q0)*np.sin(q3) - 
         np.cos(q0)*np.cos(q3)*np.sin(q2) + np.cos(q2)*np.cos(q3)*np.sin(q0)*np.sin(q1)) - 
         np.sin(q4)*(np.cos(q0)*np.cos(q2) + np.sin(q0)*np.sin(q1)*np.sin(q2)))), 

         0]
    
    A3 = [0, 
          
         d2*np.cos(q2)*np.sin(q1) - l3*np.cos(q1)*np.cos(q3) - l2*np.cos(q1) - 
         l4*np.cos(q1)*np.cos(q3)*np.cos(q5) + d3*np.cos(q1)*np.cos(q4)*np.sin(q3) + 
         l3*np.cos(q2)*np.sin(q1)*np.sin(q3) - d3*np.sin(q1)*np.sin(q2)*np.sin(q4) + 
         d3*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.sin(q1) + l4*np.cos(q2)*np.cos(q5)*np.sin(q1)*np.sin(q3) + 
         l4*np.cos(q1)*np.cos(q4)*np.sin(q3)*np.sin(q5) - l4*np.sin(q1)*np.sin(q2)*np.sin(q4)*np.sin(q5) + 
         l4*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.sin(q1)*np.sin(q5), 

         d2*np.cos(q1)*np.sin(q2) + d3*np.cos(q1)*np.cos(q2)*np.sin(q4) + 
         l3*np.cos(q1)*np.sin(q2)*np.sin(q3) + d3*np.cos(q1)*np.cos(q3)*np.cos(q4)*np.sin(q2) + 
         l4*np.cos(q1)*np.cos(q5)*np.sin(q2)*np.sin(q3) + l4*np.cos(q1)*np.cos(q2)*np.sin(q4)*np.sin(q5) + 
         l4*np.cos(q1)*np.cos(q3)*np.cos(q4)*np.sin(q2)*np.sin(q5), 

         l3*np.sin(q1)*np.sin(q3) - l3*np.cos(q1)*np.cos(q2)*np.cos(q3) + 
         d3*np.cos(q3)*np.cos(q4)*np.sin(q1) + l4*np.cos(q5)*np.sin(q1)*np.sin(q3) - 
         l4*np.cos(q1)*np.cos(q2)*np.cos(q3)*np.cos(q5) + d3*np.cos(q1)*np.cos(q2)*np.cos(q4)*np.sin(q3) + 
         l4*np.cos(q3)*np.cos(q4)*np.sin(q1)*np.sin(q5) + 
         l4*np.cos(q1)*np.cos(q2)*np.cos(q4)*np.sin(q3)*np.sin(q5), 

         d3*np.cos(q1)*np.cos(q4)*np.sin(q2) - d3*np.sin(q1)*np.sin(q3)*np.sin(q4) + 
         d3*np.cos(q1)*np.cos(q2)*np.cos(q3)*np.sin(q4) + l4*np.cos(q1)*np.cos(q4)*np.sin(q2)*np.sin(q5) - 
         l4*np.sin(q1)*np.sin(q3)*np.sin(q4)*np.sin(q5) + 
         l4*np.cos(q1)*np.cos(q2)*np.cos(q3)*np.sin(q4)*np.sin(q5), 

         l4*np.cos(q3)*np.sin(q1)*np.sin(q5) + l4*np.cos(q1)*np.cos(q2)*np.sin(q3)*np.sin(q5) + 
         l4*np.cos(q1)*np.cos(q5)*np.sin(q2)*np.sin(q4) + l4*np.cos(q4)*np.cos(q5)*np.sin(q1)*np.sin(q3) - 
         l4*np.cos(q1)*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.cos(q5), 

         0]
    
    B1 = [-np.sin(q0), 
          
         np.cos(q0)*np.sin(pi/2 + q1), 

         -np.cos(q2)*np.sin(q0) - np.cos(q0)*np.cos(pi/2 + q1)*np.sin(q2), 

         np.cos(q0)*np.cos(q3)*np.sin(pi/2 + q1) - np.sin(q3)*(np.sin(q0)*np.sin(q2) - 
         np.cos(q0)*np.cos(q2)*np.cos(pi/2 + q1)), 

         np.sin(q4)*(np.cos(q3)*(np.sin(q0)*np.sin(q2) - np.cos(q0)*np.cos(q2)*np.cos(pi/2 + q1)) + 
         np.cos(q0)*np.sin(q3)*np.sin(pi/2 + q1)) - np.cos(q4)*(np.cos(q2)*np.sin(q0) + 
         np.cos(q0)*np.cos(pi/2 + q1)*np.sin(q2)), 

         -np.cos(q5)*(np.sin(q3)*(np.sin(q0)*np.sin(q2) - np.cos(q0)*np.cos(q2)*np.cos(pi/2 + q1)) - 
         np.cos(q0)*np.cos(q3)*np.sin(pi/2 + q1)) - 
         np.sin(q5)*(np.cos(q4)*(np.cos(q3)*(np.sin(q0)*np.sin(q2) - 
         np.cos(q0)*np.cos(q2)*np.cos(pi/2 + q1)) + np.cos(q0)*np.sin(q3)*np.sin(pi/2 + q1)) + 
         np.sin(q4)*(np.cos(q2)*np.sin(q0) + np.cos(q0)*np.cos(pi/2 + q1)*np.sin(q2))), 

         -np.cos(q5)*(np.sin(q3)*(np.sin(q0)*np.sin(q2) - np.cos(q0)*np.cos(q2)*np.cos(pi/2 + q1)) - 
         np.cos(q0)*np.cos(q3)*np.sin(pi/2 + q1)) - 
         np.sin(q5)*(np.cos(q4)*(np.cos(q3)*(np.sin(q0)*np.sin(q2) - np.cos(q0)*np.cos(q2)*np.cos(pi/2 + 
         q1)) + np.cos(q0)*np.sin(q3)*np.sin(pi/2 + q1)) + np.sin(q4)*(np.cos(q2)*np.sin(q0) + 
         np.cos(q0)*np.cos(pi/2 + q1)*np.sin(q2)))]
    
    B2 = [np.cos(q0), 
          
         np.sin(q0)*np.sin(pi/2 + q1), 

         np.cos(q0)*np.cos(q2) - np.cos(pi/2 + q1)*np.sin(q0)*np.sin(q2), 

         np.sin(q3)*(np.cos(q0)*np.sin(q2) + np.cos(q2)*np.cos(pi/2 + q1)*np.sin(q0)) + 
         np.cos(q3)*np.sin(q0)*np.sin(pi/2 + q1), 

         np.cos(q4)*(np.cos(q0)*np.cos(q2) - np.cos(pi/2 + q1)*np.sin(q0)*np.sin(q2)) - 
         np.sin(q4)*(np.cos(q3)*(np.cos(q0)*np.sin(q2) + np.cos(q2)*np.cos(pi/2 + q1)*np.sin(q0)) - 
         np.sin(q0)*np.sin(q3)*np.sin(pi/2 + q1)),   

         np.cos(q5)*(np.sin(q3)*(np.cos(q0)*np.sin(q2) + np.cos(q2)*np.cos(pi/2 + q1)*np.sin(q0)) + 
         np.cos(q3)*np.sin(q0)*np.sin(pi/2 + q1)) + 
         np.sin(q5)*(np.cos(q4)*(np.cos(q3)*(np.cos(q0)*np.sin(q2) + 
         np.cos(q2)*np.cos(pi/2 + q1)*np.sin(q0)) - np.sin(q0)*np.sin(q3)*np.sin(pi/2 + q1)) + 
         np.sin(q4)*(np.cos(q0)*np.cos(q2) - np.cos(pi/2 + q1)*np.sin(q0)*np.sin(q2))), 

         np.cos(q5)*(np.sin(q3)*(np.cos(q0)*np.sin(q2) + np.cos(q2)*np.cos(pi/2 + q1)*np.sin(q0)) + 
         np.cos(q3)*np.sin(q0)*np.sin(pi/2 + q1)) + 
         np.sin(q5)*(np.cos(q4)*(np.cos(q3)*(np.cos(q0)*np.sin(q2) + 
         np.cos(q2)*np.cos(pi/2 + q1)*np.sin(q0)) - np.sin(q0)*np.sin(q3)*np.sin(pi/2 + q1)) + 
         np.sin(q4)*(np.cos(q0)*np.cos(q2) - np.cos(pi/2 + q1)*np.sin(q0)*np.sin(q2)))]
    
    B3 = [0, 
         
         np.cos(pi/2 + q1), 

         np.sin(q2)*np.sin(pi/2 + q1), 

         np.cos(q3)*np.cos(pi/2 + q1) - np.cos(q2)*np.sin(q3)*np.sin(pi/2 + q1), 

         np.sin(q4)*(np.cos(pi/2 + q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(pi/2 + q1)) + 
         np.cos(q4)*np.sin(q2)*np.sin(pi/2 + q1), 

         np.cos(q5)*(np.cos(q3)*np.cos(pi/2 + q1) - np.cos(q2)*np.sin(q3)*np.sin(pi/2 + q1)) - 
         np.sin(q5)*(np.cos(q4)*(np.cos(pi/2 + q1)*np.sin(q3) + 
         np.cos(q2)*np.cos(q3)*np.sin(pi/2 + q1)) - np.sin(q2)*np.sin(q4)*np.sin(pi/2 + q1)), 

         np.cos(q5)*(np.cos(q3)*np.cos(pi/2 + q1) - np.cos(q2)*np.sin(q3)*np.sin(pi/2 + q1)) - 
         np.sin(q5)*(np.cos(q4)*(np.cos(pi/2 + q1)*np.sin(q3) + 
         np.cos(q2)*np.cos(q3)*np.sin(pi/2 + q1)) - np.sin(q2)*np.sin(q4)*np.sin(pi/2 + q1))]
    
    J = np.matrix([A1, A2, A3, B1, B2, B3])
    
    return J