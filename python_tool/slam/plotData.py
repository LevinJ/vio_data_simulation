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

    
class PlotData(object):
    def __init__(self):
        
        self.meta_data_dict ={"lat": [],
                              "lon": [],
                              "alt": [],
                              "x": [],
                              "y": [],
                              "z": [],
                              "seq":[],
                              "stddev":[], 
                              "time_secs":[],
                              "time_nsecs":[]}
        self.meta_data_dict_imu ={"angvelx": [],
                                  "angvely": [],
                                  "angvelz": [],
                              "accelx": [],
                              "accely": [],
                              "accelz": [],
                              "seq":[],
                              "time_secs":[],
                              "time_nsecs":[]}
        self.meta_data_dict_cam ={"seq":[],
                              "time_secs":[],
                              "time_nsecs":[]}
        
        self.topic2filed = {"/navsat/fix": "bestpos_slam",
                            "/navsat/fix_nonspan": "bestgnsspos_slam",
                            "/imu/raw_data": "imu_slam",
                            "/camera/left/image": "cam_slam"}
        self.disp_inspva = False
        self.disp_gnssonly = True
        self.disp_bespos = True
        self.ignore_z = False
        self.draw2d = True  # only draw x, y coordinate
        
        return
    def process_gnsspos(self, msg):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        stddev = None
        
        utm_pos = geodesy.utm.fromLatLong(lat, lon) 
        x = utm_pos.easting
        y = utm_pos.northing
        z = alt
        time_secs = msg.header.stamp.secs
        time_nsecs = msg.header.stamp.nsecs * 1e-9
        
        stddev = np.sqrt(np.array(msg.position_covariance)).tolist()
        seq = msg.header.seq
        
        for k in self.meta_data_dict.keys():
            self.meta_data_dict[k].append(eval(k))
        return
    def process_imu(self, msg):
        
        angvelx,angvely,angvelz = [msg.angular_velocity.x, msg.angular_velocity.y,msg.angular_velocity.z]
        accelx,accely,accelz = [msg.linear_acceleration.x, msg.linear_acceleration.y,msg.linear_acceleration.z]
        time_secs = msg.header.stamp.secs
        time_nsecs = msg.header.stamp.nsecs * 1e-9
        
        
        seq = msg.header.seq
        
        for k in self.meta_data_dict_imu.keys():
            self.meta_data_dict_imu[k].append(eval(k))
        return
    def process_cam(self, msg):

        time_secs = msg.header.stamp.secs
        time_nsecs = msg.header.stamp.nsecs * 1e-9
        seq = msg.header.seq
        
        for k in self.meta_data_dict_cam.keys():
            self.meta_data_dict_cam[k].append(eval(k))
        return
    def process_msg(self,topic, msg, t, target_topic):
        
        if topic == target_topic:
            if('/navsat/fix' in topic):
                self.process_gnsspos(msg)
            elif('/camera/left/image' in topic):
                self.process_cam(msg)
            else:
                self.process_imu(msg)
                
            
       
        return
    
    def save_data(self, csv_filename, meta_data):    
        df = pd.DataFrame(meta_data)  
        df.to_csv(csv_filename)  
        for k in meta_data.keys():
            meta_data[k] = []
        return df 
    def save_gps(self, csv_filename):
        xyz = np.hstack((np.array(self.meta_data_dict["x"]).reshape(-1,1), 
                         np.array(self.meta_data_dict["y"]).reshape(-1,1), 
                         np.array(self.meta_data_dict["z"]).reshape(-1,1)))
        xyz = xyz - xyz[0]
        
        self.meta_data_dict["x"] = xyz[:,0]
        self.meta_data_dict["y"] = xyz[:,1]
        self.meta_data_dict["z"] = xyz[:,2]
        
        df = pd.DataFrame(self.meta_data_dict)  
        df.to_csv(csv_filename)  
        for k in self.meta_data_dict.keys():
            self.meta_data_dict[k] = []
        
        return df
    def on_pick(self, event):
        
        artist = event.artist
        xmouse, ymouse = event.mouseevent.xdata, event.mouseevent.ydata
        x, y = artist.get_xdata(), artist.get_ydata()
        ind = event.ind
        print 'Artist picked:', event.artist
        print '{} vertices picked'.format(len(ind))
        print 'Pick between vertices {} and {}'.format(min(ind), max(ind)+1)
        print 'x, y of mouse: {:.2f},{:.2f}'.format(xmouse, ymouse)
        print 'Data point:', x[ind[0]], y[ind[0]]
        if 'GNSS' in str(event.artist):
            df = self.df_bestpos_gnss
        else:
            df = self.df_bestpos
        print("GNSS Data: \n{}".format(df.iloc[ind[0]]))
        target_time = np.array(df[['time_secs', 'time_nsecs']].iloc[ind[0]])
        target_time = target_time.reshape(1, 2)
        
        #associated IMU data
        ds, inds = self.imutime_kdtree.query(target_time, k=1)
        print("imu ds={}".format(ds[0][0]))
        print("imu data=\n{}".format(self.df_imu.iloc[inds.flatten()[0]]))
        
        #associated camera data
        ds, inds = self.camtime_kdtree.query(target_time, k=1)
        print("cam ds={}".format(ds[0][0]))
        print("cam data=\n{}".format(self.df_cam.iloc[inds.flatten()[0]]))
        
        return
    def plot_gps2d(self, df_bespos, df_bestpos_gnss, df_inpva):
        tolerance = 3 # points
        fig, ax = plt.subplots()
        if self.disp_gnssonly:
            x = df_bestpos_gnss['x']
            y = df_bestpos_gnss['y']
#                 b_flag = x < 100000
#                 x = x[b_flag]
#                 y = y[b_flag]
            title = "gnssonly"
            ps =0
            pe = -1
            ax.plot(x[ps:pe], y[ps:pe],  c = "blue", label='GNSS only', picker=tolerance)
            ax.text(x[0], y[0], "Start", color='blue')
            ax.text(x.tolist()[-1], y.tolist()[-1], "End", color='blue')
            fig.canvas.callbacks.connect('pick_event', self.on_pick)
            
        if self.disp_bespos:
            x = df_bespos['x']
            y = df_bespos['y']
            ps =0
            pe = -1
            plt.scatter(x[ps:pe], y[ps:pe],c = "purple", label='Best Pos')
#         if self.disp_inspva:
#             x = df_inpva['x']
#             y = df_inpva['y']
#             ps =0
#             pe = -1
#             plt.plot(x[ps:pe], y[ps:pe],c = "green", label='invspva')
#             plt.title(title)
        ax.legend()
        ax.set_xlabel('X axis(East)')
        ax.set_ylabel('Y axis(North)')
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()
        return
        
    def plot_gps(self, df_bespos, df_bestpos_gnss, df_inpva):
       
        if self.draw2d:
            return self.plot_gps2d(df_bespos, df_bestpos_gnss, df_inpva)
        
        mpl.rcParams['legend.fontsize'] = 10
        
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        
        #add plot for bestpos
        if self.disp_bespos:
            x = df_bespos['x']
            y = df_bespos['y']
            z = df_bespos['z']
            if self.ignore_z:
                z = np.zeros_like(z)
            ax.plot(x, y, z, color='green', label='Best Pos')
            ax.text(x[0], y[0], z[0], "Start", color='green')
            ax.text(x.tolist()[-1], y.tolist()[-1],z.tolist()[-1], "End", color='blue')
        
        if self.disp_gnssonly:
            x = df_bestpos_gnss['x']
            y = df_bestpos_gnss['y']
            z = df_bestpos_gnss['z']
            if self.ignore_z:
                z = np.zeros_like(z)
#             ps =0
#             pe = 3700
            ax.scatter(x, y, z, color='blue', label='GNSS only', marker = '^')
#             ps =0
#             pe = 1800
#             ax.scatter(x[ps:pe], y[ps:pe],z[ps:pe], s=1, marker='^',c = "red")
#             
        
        if self.disp_inspva:
#             df_inpva = df_inpva[df_inpva['alt'] !=0]
            x = df_inpva['x']
            y = df_inpva['y']
            z = df_inpva['z']
            if self.ignore_z:
                z = np.zeros_like(z)
            ax.scatter(x, y, z, color='red', label='inspva', marker = 'o')
        
        ax.legend()
       
        ax.set_xlabel('X axis(East)')
        ax.set_ylabel('Y axis(North)')
        ax.set_zlabel('Z axis(UP)')
        plt.show()
        return
    
    def get_df(self, csv_filename, target_topic, bag):
        print("csvfilename ={}".format(csv_filename))
        if os.path.exists(csv_filename):
            df = pd.read_csv(csv_filename, index_col=0)
        else:
            for topic, msg, t in bag.read_messages():
                self.process_msg(topic, msg, t, target_topic)
            if 'best' in csv_filename:
                df = self.save_gps(csv_filename)
            elif 'imu_slam_gps.csv' in csv_filename:
                df = self.save_data(csv_filename, self.meta_data_dict_imu)
            else:
                df = self.save_data(csv_filename, self.meta_data_dict_cam)
        return df
    def cal_traj_length(self):
        num = len(self.df_bestpos['x'])
        step = 1
        inds = np.arange(0, num, step)
        if inds[-1] != num -1:
            inds = np.append(inds, num-1)
        traj_len = []
        for i in inds:
            if i==0:
                continue
            x2,y2,z2 = self.df_bestpos.iloc[i]['x'] ,self.df_bestpos.iloc[i]['y'], self.df_bestpos.iloc[i]['z']
            x1,y1,z1 = self.df_bestpos.iloc[i-1]['x'], self.df_bestpos.iloc[i-1]['y'], self.df_bestpos.iloc[i-1]['z']
            pnts_len = (x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2
            pnts_len = math.sqrt(pnts_len)
            traj_len.append(pnts_len)
        traj_len = np.array(traj_len)
            
        print("Overall trajectory length = {}, point number = {}".format(traj_len.sum(), num))
            
        return
    def save_gtpos(self, csv_filename):
        self.cal_traj_length()
        if os.path.exists(csv_filename):
            print("file {} already exists".format(csv_filename)) 
            return
        meta_data = {'timestamp':[],
                     'x':[],
                     'y':[],
                     'z':[],
                     'q_x':[],
                     'q_y':[],
                     'q_z':[],
                     'q_w':[]}
        meta_data['timestamp'] = self.df_bestpos['time_secs'].map(str)  +'.' + (self.df_bestpos[ 'time_nsecs'] * 1e9).map(int).map(str)
        meta_data['x'] = self.df_bestpos['x']
        meta_data['y'] = self.df_bestpos['y']
        meta_data['z'] = self.df_bestpos['z']
        
        meta_data_len = len(self.df_bestpos['z'])
        meta_data['q_x'] = np.zeros(meta_data_len)
        meta_data['q_y'] = np.zeros(meta_data_len)
        meta_data['q_z'] = np.zeros(meta_data_len)
        meta_data['q_w'] = np.ones(meta_data_len)
        
        df = pd.DataFrame(meta_data)  
        df.to_csv(csv_filename, header=None, sep=' ', index=False, columns=['timestamp', 'x','y','z','q_x','q_y','q_z','q_w']) 
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
        
        csv_filename = '{}/{}_{}.csv'.format(datafolder, file_name, "gt_pos")
        self.save_gtpos(csv_filename)
        
        self.plot_gps(self.df_bestpos, self.df_bestpos_gnss, None)
        
        
        return



if __name__ == "__main__":   
    obj= PlotData()
    obj.run()