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
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
    
class ExtactImgImu(PlotData):
    def __init__(self):
        PlotData.__init__(self)
        self.bridge = CvBridge()
        self.meta_data_dict_cam ={"timestamp":[],
                              "img_path":[]}
        self.meta_data_dict_imu ={ "timestamp":[],
                                  "angvelx": [],
                                  "angvely": [],
                                  "angvelz": [],
                              "accelx": [],
                              "accely": [],
                              "accelz": []}
        return
    
    
    def process_imu(self, msg):
        
        angvelx,angvely,angvelz = [msg.angular_velocity.x, msg.angular_velocity.y,msg.angular_velocity.z]
        accelx,accely,accelz = [msg.linear_acceleration.x, msg.linear_acceleration.y,msg.linear_acceleration.z]
        timestamp = "{}.{}".format(msg.header.stamp.secs,  msg.header.stamp.nsecs)
        
        for k in self.meta_data_dict_imu.keys():
            self.meta_data_dict_imu[k].append(eval(k))
        return
    def process_cam(self, msg):

        timestamp = "{}.{}".format(msg.header.stamp.secs,  msg.header.stamp.nsecs)
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
       
        img_path = '{}/imgs/{}.png'.format(self.datafolder, timestamp)
        cv2.imwrite(img_path, cv_image)
        
        for k in self.meta_data_dict_cam.keys():
            self.meta_data_dict_cam[k].append(eval(k))
    def save_data(self, csv_filename, meta_data):    
        df = pd.DataFrame(meta_data)  
        cols = ["timestamp",  "img_path"]
        if 'imu' in csv_filename:
            cols = ["timestamp",  "angvelx",  "angvely",  "angvelz","accelx", "accely","accelz"]
#             df = df[cols]
        df.to_csv(csv_filename, sep=' ', header=None, index = None, columns=cols)  
        for k in meta_data.keys():
            meta_data[k] = []
        return df 
    
    
    def run(self):   
        file_name = "1220_41"
        bag = rosbag.Bag(os.environ['HOME'] + '/bagfiles/novatel/' + file_name + ".bag")  
        datafolder = "{}/bagfiles/novatel/extracted/{}".format(os.environ['HOME'],  file_name)
        self.datafolder = datafolder
       
        if not os.path.exists(datafolder):
            os.makedirs(datafolder)
            os.makedirs(datafolder + "/imgs")
        
        
        
        dfs = []
        for topic in ["/imu/raw_data",  "/camera/left/image"]:
            csv_filename = '{}/{}_gps.csv'.format(datafolder, self.topic2filed[topic])
            df = self.get_df(csv_filename, topic, bag)  
            dfs.append(df) 

        self.df_imu, self.df_cam = dfs
#         self.imutime_kdtree = KDTree(self.df_imu[['time_secs', 'time_nsecs']], leaf_size=2) 
#         self.camtime_kdtree = KDTree(self.df_cam[['time_secs', 'time_nsecs']], leaf_size=2) 
#         self.bestpostime_kdtree = KDTree(self.df_bestpos[['time_secs', 'time_nsecs']], leaf_size=2)
#         
#         csv_filename = '{}/{}_{}.csv'.format(datafolder, file_name, "imudata")
#         self.save_imudata(csv_filename)
        
#         self.plot_gps(self.df_bestpos, self.df_bestpos_gnss, None)
        
        
        return



if __name__ == "__main__":   
    obj= ExtactImgImu()
    obj.run()