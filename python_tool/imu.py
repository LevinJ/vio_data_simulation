import tf
import numpy as np
import math
from  math import sin as sin
from math import cos as cos

class Param(object):
    def __init__(self):
        R =  [0, 0, -1,
            -1, 0, 0,
            0, 1, 0]
        self.R_bc = np.array(R).reshape((3,3))
        self.t_bc = np.array([0.05,0.04,0.03])
        
        self.imu_frequency = 200
        self.cam_frequency = 30
        self.imu_timestep = 1./self.imu_frequency
        self.cam_timestep = 1./self.cam_frequency
        self.t_start = 0.
        self.t_end = 20
     
        self.gyro_bias_sigma = 1.0e-5
        self.acc_bias_sigma = 0.0001
     
        self.gyro_noise_sigma = 0.015
        self.acc_noise_sigma = 0.019
     
        self.pixel_noise = 1
     
        self.fx = 460
        self.fy = 460
        self.cx = 255
        self.cy = 255
        self.image_w = 640
        self.image_h = 640
        return
 
    
class MotionData(object):
    def __init__(self):
        self.timestamp = None
        self.Rwb = None
        self.twb = None
        self.imu_acc = None
        self.imu_gyro = None
    
        self.imu_gyro_bias = None
        self.imu_acc_bias = None
    
        self.imu_velocity = None
        return

    
    
    
 
#euler2Rotation:   body frame to interitail frame
def euler2Rotation( eulerAngles):
    roll = eulerAngles[0]
    pitch = eulerAngles[1]
    yaw = eulerAngles[2]

    cr = cos(roll)
    sr = sin(roll)
    cp = cos(pitch)
    sp = sin(pitch)
    cy = cos(yaw)
    sy = sin(yaw)

    RIb = [ cy*cp ,   cy*sp*sr - sy*cr,   sy*sr + cy* cr*sp,
            sy*cp,    cy *cr + sy*sr*sp,  sp*sy*cr - cy*sr,
            -sp,         cp*sr,           cp*cr]
    RIb = np.array(RIb).reshape((3,3))
    return RIb;

def eulerRates2bodyRates(eulerAngles):

    roll = eulerAngles[0]
    pitch = eulerAngles[1]

    cr = cos(roll)
    sr = sin(roll);
    cp = cos(pitch)
    sp = sin(pitch)

 
    R=[  1,   0,    -sp,
            0,   cr,   sr*cp,
            0,   -sr,  cr*cp]
    R = np.array(R).reshape((3,3))
    return R;


class IMU():
    def __init__(self, _param):
        self.param = _param
        self.gyro_bias_ = np.zeros(3)
        self.acc_bias_ = np.zeros(3)
        return
    def MotionModel(self, t):
        data = MotionData()
        
        ellipse_x = 15
        ellipse_y = 20
        z = 1      
        K1 = 10
        K = math.pi/ 10
    
   
        position = np.array([ ellipse_x * math.cos( K * t) + 5, ellipse_y * math.sin( K * t) + 5,  z * math.sin( K1 * K * t ) + 5])
        dp = np.array([- K * ellipse_x * math.sin(K*t),  K * ellipse_y * math.cos(K*t), z*K1*K * math.cos(K1 * K * t)])
        K2 = K*K;
        ddp = np.array([ -K2 * ellipse_x * math.cos(K*t),  -K2 * ellipse_y * math.sin(K*t), -z*K1*K1*K2 * math.sin(K1 * K * t)])
    

        k_roll = 0.1;
        k_pitch = 0.2;
        eulerAngles = np.array([k_roll * math.cos(t) , k_pitch * math.sin(t) , K*t] ) # roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
        eulerAnglesRates = np.array([-k_roll * sin(t) , k_pitch * cos(t) , K])
    
    #    Eigen::Vector3d eulerAngles(0.0,0.0, K*t );   // roll ~ 0, pitch ~ 0, yaw ~ [0,2pi]
    #    Eigen::Vector3d eulerAnglesRates(0.,0. , K)
    
        Rwb = euler2Rotation(eulerAngles)   #body frame to world frame
        imu_gyro = eulerRates2bodyRates(eulerAngles).dot( eulerAnglesRates)  #  euler rates trans to body gyro
    
        gn =np.array([0,0,-9.81])  #  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
        imu_acc = Rwb.transpose().dot(( ddp -  gn ))  #  Rbw * Rwn * gn = gs
    
        data.imu_gyro = imu_gyro;
        data.imu_acc = imu_acc;
        data.Rwb = Rwb;
        data.twb = position;
        data.imu_velocity = dp;
        data.timestamp = t;
        return data;
            
    def run(self):
#         R =  np.array([[0, 0, -1],
#             [-1, 0, 0],
#             [0, 1, 0]])
#         ypr = np.array(tf.transformations.euler_from_matrix(R, "rzyx") )* 180/math.pi
#         
        self.MotionModel(2.0)
        return 
    
if __name__ == "__main__":
    obj = IMU(None)
    obj.run()