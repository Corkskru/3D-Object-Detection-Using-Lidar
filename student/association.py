# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Data association class with single nearest neighbor association and gating based on Mahalanobis distance
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
from scipy.stats.distributions import chi2

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import misc.params as params 

class Association:
    '''Data association class with single nearest neighbor association and gating based on Mahalanobis distance'''
    def __init__(self):
        self.association_matrix = np.matrix([])
        self.unassigned_tracks = []
        self.unassigned_meas = []
        
    def associate(self, track_list, meas_list, KF):
             
        ############
        # TODO Step 3: association:
        # - replace association_matrix with the actual association matrix based on Mahalanobis distance (see below) for all tracks and all measurements
        # - update list of unassigned measurements and unassigned tracks
        ############
        
        # the following only works for at most one track and one measurement
        self.association_matrix = np.matrix([]) # reset matrix
        self.unassigned_tracks = [] # reset lists
        self.unassigned_meas = []
         
        N = len(track_list)
        M = len(meas_list)

        self.association_matrix  = np.inf *np.ones((N, M))
        #A = self.association_matrix
        #MHD = self.MHD()
        #gating_check = self.gating()
       
  

        for i in range(N):
             track = track_list[i]
             for j in range(M):
                      #check if the measurement lies in the gating threshold of the track
                      meas = meas_list[j]
                      mhd = self.MHD(track , meas, KF)
                      sensor = meas.sensor
                      if self.gating(  mhd, sensor): 
		          #assign MHD to association matrix if gating is ok
                          self.association_matrix[i, j] = mhd
                      else:                 
                          #Set to inf and then update the unassigned meas and track lists
                          self.association_matrix[i, j] = np.inf 
                          #self.unassigned_tracks.append(track_list[i])   #No need to handle this code over here... just maintain a track list that can be easily indexed
                          #self.unassigned_meas.append(meas_list[j])    #same thing here

        #print(self.association_matrix)   
        #This step helps as a lookup list for get_closest_track_and_meas() to get indices for track_list and meas_list
        #for track,meas in zip(track_list,meas_list):
           #    self.unassigned_tracks.append(track)
              # self.unassigned_meas.append(meas)
       
        self.unassigned_tracks = np.arange(len(track_list)).tolist()
        self.unassigned_meas = np.arange(len(meas_list)).tolist()
        #self.association_matrix = np.matrix(self.association_matrix)

        return     
        
        ############
        # END student code
        ############ 
                
    def get_closest_track_and_meas(self):
        ############
        # TODO Step 3: find closest track and measurement:
        # - find minimum entry in association matrix
        # - delete row and column
        # - remove corresponding track and measurement from unassigned_tracks and unassigned_meas
        # - return this track and measurement
        ############
        
        A = self.association_matrix
        if np.min(A) == np.inf:
             return np.nan, np.nan

        #get indices of the smallest entry using unravel_index
        min_entry = np.unravel_index(np.argmin(A, axis=None), A.shape)
        track_idx = min_entry[0]
        meas_idx = min_entry[1]
        
        #delete the corr row and column
        A = np.delete(A, track_idx, 0)
        A = np.delete(A, meas_idx, 1)
        self.association_matrix = A

        update_track = self.unassigned_tracks[track_idx]
        update_meas = self.unassigned_meas[meas_idx]
        

        # remove from list
        if len(self.unassigned_tracks) > 0:
             self.unassigned_tracks.remove(update_track) 
        
        if len(self.unassigned_meas) > 0:
             self.unassigned_meas.remove(update_meas)
        #self.association_matrix = np.matrix([])
            
        ############
        # END student code
        ############ 
        return update_track, update_meas     

    def gating(self, MHD, sensor): 
        ############
        # TODO Step 3: return True if measurement lies inside gate, otherwise False
        ############
        #get the invesrse cumulative dist
        limit = chi2.ppf( params.gating_threshold, df=sensor.dim_meas)
        if MHD < limit : 
                 return True
        else:
                 return False    

        ############
        # END student code
        ############ 
        
    def MHD(self, track, meas, KF):
        ############
        # TODO Step 3: calculate and return Mahalanobis distance
        ############
              
	
        #get H       
        hx = meas.sensor.get_H(track.x)

        #get R
        R = meas.R
        
        gamma = meas.z - hx* track.x

        S = hx*track.P*hx.transpose() + R
        
        MHD = gamma.transpose() * np.linalg.inv(S) *gamma #MHD distance
        return MHD
        
        ############
        # END student code
        ############ 
    
    def associate_and_update(self, manager, meas_list, KF):
        # associate measurements and tracks
        self.associate(manager.track_list, meas_list, KF)
    
        # update associated tracks with measurements
        while self.association_matrix.shape[0]>0 and self.association_matrix.shape[1]>0:
            
            # search for next association between a track and a measurement
            ind_track, ind_meas = self.get_closest_track_and_meas()
            if np.isnan(ind_track):
                print('---no more associations---')
                break
            track = manager.track_list[ind_track]
            
            # check visibility, only update tracks in fov    
            if not meas_list[0].sensor.in_fov(track.x):
                continue
            
            # Kalman update
            print('update track', track.id, 'with', meas_list[ind_meas].sensor.name, 'measurement', ind_meas)
            KF.update(track, meas_list[ind_meas])
            
            # update score and track state 
            manager.handle_updated_track(track)
            
            # save updated track
            manager.track_list[ind_track] = track
            
        # run track management 
        manager.manage_tracks(self.unassigned_tracks, self.unassigned_meas, meas_list)
        
        for track in manager.track_list:            
            print('track', track.id, 'score =', track.score)