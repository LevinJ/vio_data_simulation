#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 15 18:18:24 2017

@author: hyj
"""
import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

np.set_printoptions(suppress = True)
filepath = '/home/levin/output/1220_41_mono_imu'



# imu_pose   imu_spline
position1 = []
quaterntions1 = []
timestamp1 = []
# data = np.loadtxt(filepath + '/1220_41_imudata_int.csv')
data = pd.read_csv(filepath + '/vio.csv',index_col= False, sep=' ',header=None, names=['timestamp','x','y','z','qx','qy','qz','qw'])
# timestamp1 = data[:,0]
# quaterntions1 = data[:,[tx_index + 6, tx_index + 3, tx_index + 4, tx_index + 5]] # qw,qx,qy,qz
position = data[['x','y','z']]




### plot 3d
fig = plt.figure()

show3d = False

start_id = 50

if show3d:
    ax = fig.gca(projection='3d')
     
      
    ax.plot(position['x'], position['y'], position['z'], label='imu_int')  
    ax.scatter(position.iloc[0][0], position.iloc[0][1],  position.iloc[0][2],color='red', label='start')
    ax.scatter(position.iloc[start_id][0], position.iloc[start_id][1],  position.iloc[start_id][2],color='red', label='start2')
     
    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
else:
    ax = fig.gca()
     
    ax.plot(position['x'], position['y'], label='imu_int')   
    
    ax.plot(position.iloc[0][0], position.iloc[0][1],  'r.', label='start')
    ax.plot(position.iloc[start_id][0], position.iloc[start_id][1],  'r.', label='start2')
  
    
    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')





plt.show()
