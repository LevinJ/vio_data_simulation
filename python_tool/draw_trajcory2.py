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
filepath = '/home/levin/workspace/vio_course/vio_data_simulation/python_tool/slam/temp/1220_41'


position_est = []
quaterntions1 = []
timestamp1 = []
# data = np.loadtxt(filepath + '/1220_41_imudata_int.csv')
data = pd.read_csv(filepath + '/1220_41_imudata_int.csv',index_col= False)
position_est = data[['x','y','z']]
position_gt = data[['gt_x','gt_y','gt_z']]


inds = np.arange(0, len(data), 500)



### plot 3d
fig = plt.figure()

show3d = False

if show3d:
    ax = fig.gca(projection='3d')
     
    ax.plot(position_gt['gt_x'], position_gt['gt_y'],  position_gt['gt_z'],label='gt')
    ax.plot(position_est['x'], position_est['y'], position_est['z'], label='imu_int')
     
#     ax.plot([position_gt.iloc[0]['gt_x']], [position_gt.iloc[0]['gt_y']],  [position_gt.iloc[0]['gt_z']],'r.', label='start')
#     ax.plot([position_gt.iloc[500]['gt_x']], [position_gt.iloc[500]['gt_y']], [position_gt.iloc[500]['gt_z']],  'r.', label='start2') 
#     
#     for ind in inds:
#         xs = [position_gt.iloc[ind]['gt_x'],position_est.iloc[ind]['x']]
#         ys = [position_gt.iloc[ind]['gt_y'],position_est.iloc[ind]['y']]
#         zs = [position_gt.iloc[ind]['gt_z'],position_est.iloc[ind]['z']]
#         ax.plot(xs, ys,zs)
      
    
    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
else:
    ax = fig.gca()
    ax.plot(position_gt['gt_x'], position_gt['gt_y'],  label='ground truth')
    ax.plot(position_est['x'], position_est['y'], label='imu_int')
     
     
#     ax.plot([position_gt.iloc[0]['gt_x']], [position_gt.iloc[0]['gt_y']],  'r.', label='start_1')
#     ax.plot([position_gt.iloc[5000]['gt_x']], [position_gt.iloc[5000]['gt_y']],  'g.', label='start_2')
     
    for ind in inds:
        xs = [position_gt.iloc[ind]['gt_x'],position_est.iloc[ind]['x']]
        ys = [position_gt.iloc[ind]['gt_y'],position_est.iloc[ind]['y']]
        ax.plot(xs, ys)
#     
    
    
    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')




plt.gca().set_aspect('equal', adjustable='box')
plt.show()
