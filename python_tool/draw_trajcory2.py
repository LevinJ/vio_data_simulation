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
filepath = '/home/levin/workspace/ros_projects/src/aiad_localization/scripts/slam/temp/1220_41'


position1 = []
quaterntions1 = []
timestamp1 = []
# data = np.loadtxt(filepath + '/1220_41_imudata_int.csv')
data = pd.read_csv(filepath + '/1220_41_imudata_int.csv',index_col= False)
# timestamp1 = data[:,0]
# quaterntions1 = data[:,[tx_index + 6, tx_index + 3, tx_index + 4, tx_index + 5]] # qw,qx,qy,qz
position1 = data[['x','y','z']]
position = data[['gt_x','gt_y','gt_z']]


inds = np.arange(0, len(data), 50)

# cam_pose_opt_o_0   cam_pose_opt_o_0
# position2 = []
# quaterntions2 = []
# timestamp2 = []
# data = np.loadtxt(filepath + '/imu_int_pose_noise.txt')
# # timestamp2 = data[:,0]
# # quaterntions2 = data[:,[tx_index + 6, tx_index + 3, tx_index + 4, tx_index + 5]] # qw,qx,qy,qz
# position2 = data[:,[tx_index, tx_index + 1, tx_index + 2]]


### plot 3d
fig = plt.figure()

show3d = False

if show3d:
    ax = fig.gca(projection='3d')
     
    ax.plot(position['gt_x'], position['gt_y'],  position['gt_z'],label='gt')
    ax.plot(position1['x'], position1['y'], position1['z'], label='imu_int')
     
    ax.plot([position.iloc[0]['gt_x']], [position.iloc[0]['gt_y']],  [position.iloc[0]['gt_z']],'r.', label='start')
    ax.plot([position.iloc[500]['gt_x']], [position.iloc[500]['gt_y']], [position.iloc[500]['gt_z']],  'r.', label='start2') 
      
    
    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
else:
    ax = fig.gca()
    ax.plot(position['gt_x'], position['gt_y'],  label='gt')
    ax.plot(position1['x'], position1['y'], label='imu_int')
     
     
    ax.plot([position.iloc[0]['gt_x']], [position.iloc[0]['gt_y']],  'r.', label='start')
    ax.plot([position.iloc[500]['gt_x']], [position.iloc[500]['gt_y']],  'r.', label='start2')
    
    for ind in inds:
        xs = [position.iloc[ind]['gt_x'],position1.iloc[ind]['x']]
        ys = [position.iloc[ind]['gt_y'],position1.iloc[ind]['y']]
        ax.plot(xs, ys)
    
    
    
    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')





plt.show()
