#!/usr/bin/env python3

"""
Compute state space kinematic matrices for xArm7 robot arm (5 links, 7 joints)
"""

import numpy as np
import math 

class xArm7_kinematics():
    def __init__(self):

        self.l1 = 0.267
        self.l2 = 0.293
        self.l3 = 0.0525
        self.l4 = 0.3512
        self.l5 = 0.1232

        self.theta1 = 0.2225 #(rad) (=12.75deg)
        self.theta2 = 0.6646 #(rad) (=38.08deg)

        pass

    def compute_jacobian(self, r_joints_array):

        q1=r_joints_array[0]
        q2=r_joints_array[1]
        q3=r_joints_array[2]
        q4=r_joints_array[3]
        q5=r_joints_array[4]
        q6=r_joints_array[5]
        q7=r_joints_array[6]

        theta1 = self.theta1
        theta2 = self.theta2

        l1 = self.l1
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4
        l5 = self.l5
        
        c3  = math.cos(q3)
        c4  = math.cos(q4)
        c5  = math.cos(q5)
        c6  = math.cos(q6)
        c7  = math.cos(q7)

        s3  = math.sin(q3)
        s4  = math.sin(q4)
        s5  = math.sin(q5)
        s6  = math.sin(q6)
        s7  = math.sin(q7)
        
        
        s12 = math.sin(q1+q2)
        c12 = math.cos(q1+q2)

        x= l4*math.sin(theta1) #x
        y= l4*math.cos(theta1) #y
        z= l5*math.sin(theta2) #z
        w= l5*math.cos(theta2) #w


        J_11 = y*(c12*c4 - s12*c3*s4) - l2*c12 - w*(c6*(c12*c4 - s12*c3*s4) + s6*(c5*(c12*s4 + s12*c3*c4) + s12*s3*s5)) + z*c7*(s6*(c12*c4 - s12*c3*s4) - c6*(c5*(c12*s4 + s12*c3*c4) + s12*s3*s5)) - z*s7*(s5*(c12*s4 + s12*c3*c4) - s12*c5*s3) - l3*c12*s4 - x*c5*(c12*s4 + s12*c3*c4) - l3*s12*c3*c4 - x*s12*s3*s5

        J_12 = y*(c12*c4 - s12*c3*s4) - l2*c12 - w*(c6*(c12*c4 - s12*c3*s4) + s6*(c5*(c12*s4 + s12*c3*c4) + s12*s3*s5)) + z*c7*(s6*(c12*c4 - s12*c3*s4) - c6*(c5*(c12*s4 + s12*c3*c4) + s12*s3*s5)) - z*s7*(s5*(c12*s4 + s12*c3*c4) - s12*c5*s3) - l3*c12*s4 - x*c5*(c12*s4 + s12*c3*c4) - l3*s12*c3*c4 - x*s12*s3*s5

        J_13 = -c12*(z*s7*(c3*c5 + c4*s3*s5) - w*(s6*(c3*s5 - c4*c5*s3) + c6*s3*s4) + l3*c4*s3 - x*c3*s5 + y*s3*s4 - z*c7*(c6*(c3*s5 - c4*c5*s3) - s3*s4*s6) + x*c4*c5*s3)

        J_14 = y*c12*c3*c4 - y*s12*s4 - l3*s12*c4 - l3*c12*c3*s4 - x*s12*c4*c5 + w*s12*c6*s4 - w*c12*c3*c4*c6 - x*c12*c3*c5*s4 - w*s12*c4*c5*s6 - z*s12*c4*s5*s7 - z*s12*c7*s4*s6 - z*c12*c3*s4*s5*s7 + z*c12*c3*c4*c7*s6 - z*s12*c4*c5*c6*c7 - w*c12*c3*c5*s4*s6 - z*c12*c3*c5*c6*c7*s4

        J_15 = x*c12*c5*s3 + x*s12*s4*s5 - x*c12*c3*c4*s5 + w*c12*c5*s3*s6 + z*c12*s3*s5*s7 - z*s12*c5*s4*s7 + w*s12*s4*s5*s6 + z*s12*c6*c7*s4*s5 + z*c12*c3*c4*c5*s7 + z*c12*c5*c6*c7*s3 - w*c12*c3*c4*s5*s6 - z*c12*c3*c4*c6*c7*s5

        J_16 = (c3*c5 + c4*s3*s5)*(w*(c6*(c12*c4 - s12*c3*s4) + s6*(c5*(c12*s4 + s12*c3*c4) + s12*s3*s5)) - z*c7*(s6*(c12*c4 - s12*c3*s4) - c6*(c5*(c12*s4 + s12*c3*c4) + s12*s3*s5)) + z*s7*(s5*(c12*s4 + s12*c3*c4) - s12*c5*s3)) + (s5*(c12*s4 + s12*c3*c4) - s12*c5*s3)*(w*(s6*(c3*s5 - c4*c5*s3) + c6*s3*s4) - z*s7*(c3*c5 + c4*s3*s5) + z*c7*(c6*(c3*s5 - c4*c5*s3) - s3*s4*s6))

        J_17 = (c6*(c12*c4 - s12*c3*s4) + s6*(c5*(c12*s4 + s12*c3*c4) + s12*s3*s5))*(w*(s6*(c3*s5 - c4*c5*s3) + c6*s3*s4) - z*s7*(c3*c5 + c4*s3*s5) + z*c7*(c6*(c3*s5 - c4*c5*s3) - s3*s4*s6)) - (s6*(c3*s5 - c4*c5*s3) + c6*s3*s4)*(w*(c6*(c12*c4 - s12*c3*s4) + s6*(c5*(c12*s4 + s12*c3*c4) + s12*s3*s5)) - z*c7*(s6*(c12*c4 - s12*c3*s4) - c6*(c5*(c12*s4 + s12*c3*c4) + s12*s3*s5)) + z*s7*(s5*(c12*s4 + s12*c3*c4) - s12*c5*s3))


        
        J_21 = y*(s12*c4 + c12*c3*s4) - l2*s12 - w*(c6*(s12*c4 + c12*c3*s4) + s6*(c5*(s12*s4 - c12*c3*c4) - c12*s3*s5)) + z*c7*(s6*(s12*c4 + c12*c3*s4) - c6*(c5*(s12*s4 - c12*c3*c4) - c12*s3*s5)) - z*s7*(s5*(s12*s4 - c12*c3*c4) + c12*c5*s3) - l3*s12*s4 - x*c5*(s12*s4 - c12*c3*c4) + l3*c12*c3*c4 + x*c12*s3*s5

        J_22 = y*(s12*c4 + c12*c3*s4) - l2*s12 - w*(c6*(s12*c4 + c12*c3*s4) + s6*(c5*(s12*s4 - c12*c3*c4) - c12*s3*s5)) + z*c7*(s6*(s12*c4 + c12*c3*s4) - c6*(c5*(s12*s4 - c12*c3*c4) - c12*s3*s5)) - z*s7*(s5*(s12*s4 - c12*c3*c4) + c12*c5*s3) - l3*s12*s4 - x*c5*(s12*s4 - c12*c3*c4) + l3*c12*c3*c4 + x*c12*s3*s5

        J_23 = -s12*(z*s7*(c3*c5 + c4*s3*s5) - w*(s6*(c3*s5 - c4*c5*s3) + c6*s3*s4) + l3*c4*s3 - x*c3*s5 + y*s3*s4 - z*c7*(c6*(c3*s5 - c4*c5*s3) - s3*s4*s6) + x*c4*c5*s3)

        J_24 = l3*c12*c4 + y*c12*s4 + x*c12*c4*c5 - w*c12*c6*s4 + y*s12*c3*c4 - l3*s12*c3*s4 - w*s12*c3*c4*c6 + w*c12*c4*c5*s6 - x*s12*c3*c5*s4 + z*c12*c4*s5*s7 + z*c12*c7*s4*s6 + z*s12*c3*c4*c7*s6 - w*s12*c3*c5*s4*s6 - z*s12*c3*s4*s5*s7 + z*c12*c4*c5*c6*c7 - z*s12*c3*c5*c6*c7*s4

        J_25 = x*s12*c5*s3 - x*c12*s4*s5 - x*s12*c3*c4*s5 + z*c12*c5*s4*s7 + w*s12*c5*s3*s6 - w*c12*s4*s5*s6 + z*s12*s3*s5*s7 + z*s12*c3*c4*c5*s7 + z*s12*c5*c6*c7*s3 - z*c12*c6*c7*s4*s5 - w*s12*c3*c4*s5*s6 - z*s12*c3*c4*c6*c7*s5

        J_26 = (c3*c5 + c4*s3*s5)*(w*(c6*(s12*c4 + c12*c3*s4) + s6*(c5*(s12*s4 - c12*c3*c4) - c12*s3*s5)) - z*c7*(s6*(s12*c4 + c12*c3*s4) - c6*(c5*(s12*s4 - c12*c3*c4) - c12*s3*s5)) + z*s7*(s5*(s12*s4 - c12*c3*c4) + c12*c5*s3)) + (s5*(s12*s4 - c12*c3*c4) + c12*c5*s3)*(w*(s6*(c3*s5 - c4*c5*s3) + c6*s3*s4) - z*s7*(c3*c5 + c4*s3*s5) + z*c7*(c6*(c3*s5 - c4*c5*s3) - s3*s4*s6))

        J_27 = (c6*(s12*c4 + c12*c3*s4) + s6*(c5*(s12*s4 - c12*c3*c4) - c12*s3*s5))*(w*(s6*(c3*s5 - c4*c5*s3) + c6*s3*s4) - z*s7*(c3*c5 + c4*s3*s5) + z*c7*(c6*(c3*s5 - c4*c5*s3) - s3*s4*s6)) - (s6*(c3*s5 - c4*c5*s3) + c6*s3*s4)*(w*(c6*(s12*c4 + c12*c3*s4) + s6*(c5*(s12*s4 - c12*c3*c4) - c12*s3*s5)) - z*c7*(s6*(s12*c4 + c12*c3*s4) - c6*(c5*(s12*s4 - c12*c3*c4) - c12*s3*s5)) + z*s7*(s5*(s12*s4 - c12*c3*c4) + c12*c5*s3))

        J_31 = 0
        J_32 = 0
        J_33 = w*c3*c6*s4 - y*c3*s4 - x*s3*s5 - x*c3*c4*c5 - l3*c3*c4 + z*c5*s3*s7 - w*s3*s5*s6 - w*c3*c4*c5*s6 - z*c3*c4*s5*s7 - z*c3*c7*s4*s6 - z*c6*c7*s3*s5 - z*c3*c4*c5*c6*c7

        J_34 = s3*(l3*s4 - y*c4 + w*c4*c6 + x*c5*s4 - z*c4*c7*s6 + w*c5*s4*s6 + z*s4*s5*s7 + z*c5*c6*c7*s4)

        J_35 = x*c3*c5 + w*c3*c5*s6 + x*c4*s3*s5 + z*c3*s5*s7 + z*c3*c5*c6*c7 - z*c4*c5*s3*s7 + w*c4*s3*s5*s6 + z*c4*c6*c7*s3*s5

        J_36 = w*c3*c6*s5 - w*s3*s4*s6 - w*c4*c5*c6*s3 - z*c6*c7*s3*s4 - z*c3*c7*s5*s6 + z*c4*c5*c7*s3*s6

        J_37 = -z*(c3*c5*c7 + c4*c7*s3*s5 + c3*c6*s5*s7 - s3*s4*s6*s7 - c4*c5*c6*s3*s7)

        J = np.matrix([ [ J_11 , J_12 , J_13 , J_14 , J_15 , J_16 , J_17 ],\
                        [ J_21 , J_22 , J_23 , J_24 , J_25 , J_26 , J_27 ],\
                        [ J_31 , J_32 , J_33 , J_34 , J_35 , J_36 , J_37 ]])
        return J

    def tf_A01(self, r_joints_array):

        q1 = r_joints_array[0]

        c1 = math.cos(q1)
        s1 = math.sin(q1)
        
        l1 = self.l1
        
        tf = np.matrix([[c1 ,-s1 , 0 , 0],\
                        [s1 , c1 , 0 , 0],\
                        [0  , 0  , 1 , l1],\
                        [0  , 0  , 0 , 1]])
        
        return tf

    def tf_A02(self, r_joints_array):

        q2 = r_joints_array[1]

        c2 = math.cos(q2)
        s2 = math.sin(q2)

        l2 = self.l2


        tf_A12 = np.matrix([[c2 , 0, -s2 , 0],\
                            [s2 , 0,  c2 , 0],\
                            [0  ,-1,  0  , 0],\
                            [0  , 0,  0  , 1]])
        tf = np.dot( self.tf_A01(r_joints_array), tf_A12 )
        
        return tf

    def tf_A03(self, r_joints_array):

        q3 = r_joints_array[2]

        c3 = math.cos(q3)
        s3 = math.sin(q3)

        tf_A23 = np.matrix([[c3 , 0,  s3 , 0],\
                            [s3 , 0, -c3 , 0],\
                            [0  , 1,  0  , l2],\
                            [0  , 0,  0  , 1]])
        tf = np.dot( self.tf_A02(r_joints_array), tf_A23 )
        return tf

    def tf_A04(self, r_joints_array):

        q4 = r_joints_array[3]

        c4 = math.cos(q4)
        s4 = math.sin(q4)

        l3 = self.l3

        tf_A34 = np.matrix([[c4 , 0,  s4 , l3*c4],\
                            [s4 , 0, -c4 , l3*s4],\
                            [0  , 1,  0  , 0],\
                            [0  , 0,  0  , 1]])
        tf = np.dot( self.tf_A03(r_joints_array), tf_A34 )
        return tf

    def tf_A05(self, r_joints_array):

        q5 = r_joints_array[4]

        c5 = math.cos(q5)
        s5 = math.sin(q5)

        l4 = self.l4
        theta1 = self.theta1

        l4sin_th1 = l4*math.sin(theta1)
        l4cos_th1 = l4*math.cos(theta1)


        tf_A45 = np.matrix([[c5 , 0,  s5 , l4sin_th1*c5],\
                            [s5 , 0, -c5 , l4sin_th1*s5],\
                            [0  , 1,  0  , l4cos_th1],\
                            [0  , 0,  0  , 1]])
        tf = np.dot( self.tf_A04(r_joints_array), tf_A45 )
        return tf

    def tf_A06(self, r_joints_array):


        q6 = r_joints_array[5]

        c6 = math.cos(q6)
        s6 = math.sin(q6)
        
        tf_A56 = np.matrix([[c6 , 0,  s6 , 0],\
                            [s6 , 0, -c6 , 0],\
                            [0  , 1,  0  , 0],\
                            [0  , 0,  0  , 1]])
        tf = np.dot( self.tf_A05(r_joints_array), tf_A56 )
        return tf

    def tf_A07(self, r_joints_array):


        q7 = r_joints_array[6]

        c7 = math.cos(q7)
        s7 = math.sin(q7)
        
        l5 = self.l5
        theta2 = self.theta2

        l5sin_th2 = l5*math.sin(theta2)
        l5cos_th2 = l5*math.cos(theta2)

        tf_A67 = np.matrix([[c7 , 0, -s7 , l5sin_th2*c7],\
                            [s7 , 0,  c7 , l5sin_th2*s7],\
                            [0  ,-1,  0  , l5cos_th2],\
                            [0  , 0,  0  , 1]])
        tf = np.dot( self.tf_A06(r_joints_array), tf_A67 )
        return tf
