#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <sys/stat.h>
#include "imu.h"
#include "utilities.h"


using namespace std;



extern "C" {

void propagate(char * src, char * dist, double x, double y, double z,
		double roll, double pitch, double yaw, double vx, double vy, double vz){
	cout<<"hello world"<<endl;
	cout<<src<<endl;
	cout<<dist<<endl;
	cout<<x<<","<<y<<","<<z<<endl;
	cout<<roll<<","<<pitch<<","<<yaw<<endl;
	cout<<vx<<","<<vy<<","<<vz<<endl;


	// IMU model
	Param params;
	IMU imuGen(params);

	std::vector<MotionData>imudata;
	LoadPose(src,imudata);

	std::ofstream save_points;
	save_points.open(dist);

	Eigen::Vector3d dp(vx,  vy, vz);
	Eigen::Vector3d position( x, y,  z);
	Eigen::Vector3d eulerAngles(roll , pitch, yaw );
	Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles);

	imuGen.init_velocity_ = dp;
	imuGen.init_twb_ = position;
	imuGen.init_Rwb_ = Rwb;

	double dt = params.imu_timestep;
	Eigen::Vector3d Pwb = imuGen.init_twb_;              // position :    from  imu measurements
	Eigen::Quaterniond Qwb(imuGen.init_Rwb_);            // quaterniond:  from imu measurements
	Eigen::Vector3d Vw = imuGen.init_velocity_;          // velocity  :   from imu measurements
	Eigen::Vector3d gw(0,0,-9.81);    // ENU frame
	Eigen::Vector3d temp_a;
	Eigen::Vector3d theta;
	for (int i = 1; i < imudata.size(); ++i) {

		MotionData imupose = imudata[i];

		//delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
  /*      Eigen::Quaterniond dq;
		Eigen::Vector3d dtheta_half =  imupose.imu_gyro * dt /2.0;
		dq.w() = 1;
		dq.x() = dtheta_half.x();
		dq.y() = dtheta_half.y();
		dq.z() = dtheta_half.z();
		dq.normalize();

		/// imu 动力学模型 欧拉积分
		Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw
		Qwb = Qwb * dq;
		Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
		Vw = Vw + acc_w * dt;
		*/


		/// 中值积分
		Eigen::Quaterniond dq;
		Eigen::Vector3d omega;
		omega = 1.0/2 * (imupose.imu_gyro + imudata[i-1].imu_gyro);

		Eigen::Vector3d dtheta_half =  omega * dt /2.0;
		dq.w() = 1;
		dq.x() = dtheta_half.x();
		dq.y() = dtheta_half.y();
		dq.z() = dtheta_half.z();
		dq.normalize();

		Eigen::Vector3d acc_w;
		Eigen::Quaterniond Qwb_next = Qwb * dq;
		acc_w = 1.0/2 *(Qwb * (imupose.imu_acc) + Qwb_next * (imudata[i-1].imu_acc)) + gw;

		/// imu 动力学模型, consant turn rate, constant acceleration, CTRA
		Qwb = Qwb * dq;
		Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
		Vw = Vw + acc_w * dt;




		//　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
		save_points<<imupose.timestamp<<" "
				   <<Qwb.w()<<" "
				   <<Qwb.x()<<" "
				   <<Qwb.y()<<" "
				   <<Qwb.z()<<" "
				   <<Pwb(0)<<" "
				   <<Pwb(1)<<" "
				   <<Pwb(2)<<" "
				   <<Qwb.w()<<" "
				   <<Qwb.x()<<" "
				   <<Qwb.y()<<" "
				   <<Qwb.z()<<" "
				   <<Pwb(0)<<" "
				   <<Pwb(1)<<" "
				   <<Pwb(2)<<" "
				   <<std::endl;

	}

	std::cout<<"test　end"<<std::endl;



	return;
}

}
