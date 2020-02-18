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
from  sklearn.neighbors import KDTree
np.set_printoptions(suppress = True)
# filepath = '/home/levin/output/1220_41_mono_imu'
# filepath = '/home/levin/output/1220_41_stereo'
filepath = '/home/levin/output'



# imu_pose   imu_spline
position1 = []
quaterntions1 = []
timestamp1 = []
# data = np.loadtxt(filepath + '/1220_41_imudata_int.csv')
position = pd.read_csv(filepath + '/vio.csv',index_col= False, sep=' ',header=None, names=['timestamp','x','y','z','qx','qy','qz','qw'])



filepath = '/home/levin/workspace/vio_course/vio_data_simulation/python_tool/slam/temp/1220_41/1220_41_gt_pos.csv'
gt_position = pd.read_csv(filepath,index_col= False, sep=' ',header=None, names=['timestamp','x','y','z','qx','qy','qz','qw'])



gtpos_kdtree = KDTree(np.array(gt_position['timestamp']).reshape(-1,1), leaf_size=2)
target_time = np.array(position['timestamp']).reshape(-1,1)
ds, inds = gtpos_kdtree.query(target_time, k=1)

gtpos_inds = inds.flatten()
pos_inds = np.arange(len(position))

inds = np.arange(0, len(pos_inds), 15)

### plot 3d
fig = plt.figure()

show3d = False

start_id = 200

if show3d:
    ax = fig.gca(projection='3d')
     
      
    ax.plot(position['x'], position['y'], position['z'], label='slam')  
    ax.scatter(position.iloc[0][0], position.iloc[0][1],  position.iloc[0][2],color='red', label='start')
    ax.scatter(position.iloc[start_id][0], position.iloc[start_id][1],  position.iloc[start_id][2],color='green', label='start2')
     
    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
else:
    ax = fig.gca()
     
    ax.plot(position['x'], position['y'], label='slam')   
#     ax.plot(position.iloc[0]['x'], position.iloc[0]['y'],  'r.', label='start_1')
#     ax.plot(position.iloc[start_id]['x'], position.iloc[start_id]['y'],  'g.', label='start_2')
    
    ax.plot(gt_position['x'], gt_position['y'], label='ground truth')   
#     ax.plot(gt_position.iloc[0]['x'], gt_position.iloc[0]['y'],  'r.', label='start_1')
#     ax.plot(gt_position.iloc[start_id]['x'], gt_position.iloc[start_id]['y'],  'g.', label='start_2')
    for ind, gt_ind in zip(pos_inds, gtpos_inds):
        if not ind in inds:
            continue
        xs = [gt_position.iloc[gt_ind]['x'],position.iloc[ind]['x']]
        ys = [gt_position.iloc[gt_ind]['y'],position.iloc[ind]['y']]
        ax.plot(xs, ys)

    
    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')




plt.gca().set_aspect('equal', adjustable='box')
plt.show()
