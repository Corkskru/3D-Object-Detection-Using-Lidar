# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
         self.dim_state = params.dim_state
         self.dt = params.dt
         self.q = params.q

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############
        #linear measurement in 3d-space .. hence use a 6d matrix
        F = np.matrix(np.identity(self.dim_state))
        F[0, 3] = self.dt
        F[1, 4] = self.dt
        F[2, 5] = self.dt
        
        return F
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############
        Q = np.matrix(np.zeros( (self.dim_state, self.dim_state) ))
        q = self.q * self.dt
        np.fill_diagonal(Q, q)
        return Q
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############
        x_predict = self.F()*track.x
        P_predict = self.F()*track.P*self.F().transpose() + self.Q() #covariance prediction
        track.set_x(x_predict)
        track.set_P(P_predict)
        
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        H = meas.sensor.get_H(track.x)
        S = self.S(track ,meas, H)
        gamma = self.gamma(track, meas)
        K = track.P*H.transpose() * np.linalg.inv(S) # Kalman gain
        x = track.x + K*gamma                               #state update
        I  =  np.identity(self.dim_state)
        P = (I - K*H)  * track.P    #co-variance update
      
        #update x and P
        track.set_x(x)
        track.set_P(P)
        
        
         
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############
        H = meas.sensor.get_hx(track.x)
        gamma = meas.z - H
        return gamma
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############
        S = H*track.P*H.transpose() + meas.R  # covariance of residual
        return S
        
        ############
        # END student code
        ############ 