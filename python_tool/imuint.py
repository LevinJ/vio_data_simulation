from  ctypes import *
import time
import os




class IMUInt(object):
    def __init__(self):
        lib = cdll.LoadLibrary(os.path.abspath(os.path.join(os.path.dirname(__file__), 
                                                            '../lib/libimu_integration.so')))

        lib.propagate.argtypes = [c_char_p,c_char_p,c_double,c_double,c_double,c_double,c_double,c_double,c_double,c_double,c_double]
        lib.propagate.restype = None
      
        
        self.lib = lib
        
        return
   
    def run(self):
        self.lib.propagate("/home/levin/workspace/vio_course/vio_data_simulation/bin/imu_pose.txt", 
                           "/home/levin/workspace/vio_course/vio_data_simulation/bin/imu_int_pose3.txt", 
                           20,5,5,
                           0.1,0,0,
                           0,6.2831854820251465,3.1415927410125732)
        return
    
g_imuint = IMUInt()
if __name__ == "__main__":   
    obj = g_imuint
    obj.run()
    




