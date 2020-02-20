import rosbag
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import geodesy.utm
import os
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from  sklearn.neighbors import KDTree
import math
pd.set_option('display.max_colwidth', 100)
from plotData import PlotData
    
class ExtactIMUData(PlotData):
    
    def save_imudata(self, csv_filename):
        self.cal_traj_length()
        if os.path.exists(csv_filename):
            print("file {} already exists".format(csv_filename)) 
            return
#         min_id = 7745
#         max_Id = 9525
#         min_id = 0
#         max_Id = 12194
        
        #first straight line    
#         min_id = 3940
#         max_Id = 7745
        #same as vins
        min_id = 0
        max_Id = 12194
        
        
        df = self.df_imu[(self.df_imu['seq']>min_id) & (self.df_imu['seq']<max_Id)].reset_index(drop=True)
        
        
        df_len = len(self.df_bestpos)
        self.df_bestpos['qw'] = np.ones(df_len)
        self.df_bestpos['qx'] = np.zeros(df_len)
        self.df_bestpos['qy'] = np.zeros(df_len)
        self.df_bestpos['qz'] = np.zeros(df_len)
        
        self.df_bestpos['roll'] = np.zeros(df_len)
        self.df_bestpos['pitch'] = np.zeros(df_len)
        
        x1 = np.array(self.df_bestpos['x'])
        y1 = np.array(self.df_bestpos['y'])
        z1 = np.array(self.df_bestpos['z'])
        
        plt.scatter(x1, y1)
        
        x2 = x1[1:]
        y2 = y1[1:]

        
        x1 = x1[:-1]
        y1 = y1[:-1]
        
        yaw = np.arctan2(y2 - y1, x2 - x1)*180/3.14
        yaw = np.append(yaw, [yaw[-1]])
        
        dt = self.df_bestpos['time_secs'][1:].reset_index(drop=True) - self.df_bestpos['time_secs'][:-1]  + self.df_bestpos['time_nsecs'][1:].reset_index(drop=True) - self.df_bestpos['time_nsecs'][:-1]
        
        v_df= (self.df_bestpos[['x','y', 'z']][1:].reset_index(drop=True) - self.df_bestpos[['x','y', 'z']][:-1])
        v_df = v_df.div(dt, axis=0)
        
        v_df = v_df.append(v_df.iloc[-1], ignore_index=True)
        
        
    
        
        self.df_bestpos['vx'] = v_df ['x']
        self.df_bestpos['vy'] = v_df ['y']
        self.df_bestpos['vz'] = v_df ['z']
        
        
        a_df= (v_df[['x','y', 'z']][1:].reset_index(drop=True) - v_df[['x','y', 'z']][:-1])
        a_df = a_df.div(dt, axis=0)
        a_df = a_df.append(a_df.iloc[-1], ignore_index=True)
        
        self.df_bestpos['ax'] = a_df ['x']
        self.df_bestpos['ay'] = a_df ['y']
        self.df_bestpos['az'] = a_df ['z']
        
        
        self.df_bestpos['yaw'] = yaw
        
        target_time = df[['time_secs', 'time_nsecs']]
        
        #associated IMU data
        ds, inds = self.bestpostime_kdtree.query(target_time, k=1)
        df_pose = self.df_bestpos.iloc[inds.flatten()]
        df_pose = df_pose[['x', 'y', 'z', 'qw','qx', 'qy', 'qz', 'roll', 'pitch', 'yaw', 'vx','vy','vz', 'ax', 'ay', 'az']]
        
        df = pd.concat([df, df_pose.reset_index(drop=True)], axis = 1)
#         df['time_stamp'] = df['time_secs'] - df['time_secs'][0] + df['time_nsecs']
        df['time_stamp'] = df['time_secs']  + df['time_nsecs']

        cols =['time_stamp', 'qw','qx','qy','qz','x','y','z', 'angvelx','angvely', 'angvelz','accelx' ,'accely' ,'accelz' ,'roll', 'pitch', 'yaw' ,'seq','time_secs','time_nsecs' ,'vx', 'vy' ,'vz', 'ax', 'ay', 'az']
        df = df[cols]
        df.to_csv(csv_filename,  sep=' ', index=False, columns=cols) 
#         df.to_csv(csv_filename) 
        print("file {} saved".format(csv_filename)) 
        return
    
    def run(self):   
        file_name = "1220_41"
        bag = rosbag.Bag(os.environ['HOME'] + '/bagfiles/novatel/' + file_name + ".bag")  
        datafolder = os.path.abspath(os.path.join(os.path.dirname(__file__), 'temp/'))
        datafolder = '{}/{}'.format(datafolder, file_name)
        if not os.path.exists(datafolder):
            os.makedirs(datafolder)
        
        
        
        dfs = []
        for topic in ["/navsat/fix", "/navsat/fix_nonspan", "/imu/raw_data", "/camera/left/image"]:
            csv_filename = '{}/{}_gps.csv'.format(datafolder, self.topic2filed[topic])
            df = self.get_df(csv_filename, topic, bag)  
            dfs.append(df) 

        self.df_bestpos, self.df_bestpos_gnss,self.df_imu, self.df_cam = dfs
        self.imutime_kdtree = KDTree(self.df_imu[['time_secs', 'time_nsecs']], leaf_size=2) 
        self.camtime_kdtree = KDTree(self.df_cam[['time_secs', 'time_nsecs']], leaf_size=2) 
        self.bestpostime_kdtree = KDTree(self.df_bestpos[['time_secs', 'time_nsecs']], leaf_size=2)
        
        csv_filename = '{}/{}_{}.csv'.format(datafolder, file_name, "imudata")
        self.save_imudata(csv_filename)
        
#         self.plot_gps(self.df_bestpos, self.df_bestpos_gnss, None)
        
        
        return



if __name__ == "__main__":   
    obj= ExtactIMUData()
    obj.run()