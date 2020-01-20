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

# imu_circle   imu_spline
position = []
quaterntions = []
timestamp = []
tx_index = 5
position = np.loadtxt(filepath + '/1220_41_imudata.csv', usecols = (tx_index, tx_index + 1, tx_index + 2))

# imu_pose   imu_spline
position1 = []
quaterntions1 = []
timestamp1 = []
# data = np.loadtxt(filepath + '/1220_41_imudata_int.csv')
data = pd.read_csv(filepath + '/1220_41_imudata_int.csv',index_col= False)
# timestamp1 = data[:,0]
# quaterntions1 = data[:,[tx_index + 6, tx_index + 3, tx_index + 4, tx_index + 5]] # qw,qx,qy,qz
position1 = data[['x','y','z']]

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
ax = fig.gca(projection='3d')

ax.plot(position[:,0], position[:,1], position[:,2], label='gt')
ax.plot(position1['x'], position1['y'], position1['z'], label='imu_int')
# ax.plot(position2[:,0], position2[:,1], position2[:,2], label='noise')
ax.plot([position[0,0]], [position[0,1]], [position[0,2]], 'r.', label='start')
ax.plot([position[500,0]], [position[500,1]], [position[500,2]], 'r.', label='start2')

ax.legend()
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()
