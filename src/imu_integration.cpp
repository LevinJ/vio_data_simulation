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

static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / M_PI * 180.0;
    }
Eigen::Matrix3d ypr2R(const Eigen::Vector3d &ypr)
    {
//        typedef typename double Scalar_t;

        double y = ypr(0) / 180.0 * M_PI;
        double p = ypr(1) / 180.0 * M_PI;
        double r = ypr(2) / 180.0 * M_PI;

        Eigen::Matrix<double, 3, 3> Rz;
        Rz << cos(y), -sin(y), 0,
            sin(y), cos(y), 0,
            0, 0, 1;

        Eigen::Matrix<double, 3, 3> Ry;
        Ry << cos(p), 0., sin(p),
            0., 1., 0.,
            -sin(p), 0., cos(p);

        Eigen::Matrix<double, 3, 3> Rx;
        Rx << 1., 0., 0.,
            0., cos(r), -sin(r),
            0., sin(r), cos(r);

        return Rz * Ry * Rx;
    }

void propagate(char * src, char * dist, double x, double y, double z,
		double roll, double pitch, double yaw, double vx, double vy, double vz){


	//force xy plane movement
//	z=  0;
//	vz= 0 ;


	cout<<"hello world"<<endl;
	cout<<src<<endl;
	cout<<dist<<endl;
	cout<<x<<","<<y<<","<<z<<endl;
	cout<<roll<<","<<pitch<<","<<yaw*180/3.14<<endl;
	cout<<vx<<","<<vy<<","<<vz<<endl;


	// IMU model
	Param params;
	IMU imuGen(params);

	std::vector<MotionData>imudata;
	LoadPose(src,imudata);

	std::ofstream save_points;
	save_points.open(dist);
	//write the header
	save_points<<"timestamp"<<","

			 	 	   <<"roll"<<","
					   <<"pitch"<<","
					   <<"yaw"<<","
				   	   <<"gt_roll"<<","
					   <<"gt_pitch"<<","
					   <<"gt_yaw"<<","

					   <<"x"<<","
					   <<"y"<<","
					   <<"z"<<","
					   <<"gt_x"<<","
					   <<"gt_y"<<","
					   <<"gt_z"<<","

					   <<"wx_b"<<","
					   <<"wy_b"<<","
					   <<"wz_b"<<","
					   <<"ax_b"<<","
					   <<"ay_b"<<","
					   <<"az_b"<<","
					   <<"ax_w"<<","
					   <<"ay_w"<<","
					   <<"az_w"<<","
					   <<"vx_w"<<","
					   <<"vy_w"<<","
					   <<"vz_w"<<","
					   <<"seq"<<","



					   <<std::endl;

	Eigen::Vector3d dp(vx,  vy, vz);
	Eigen::Vector3d position( x, y,  z);
	Eigen::Vector3d eulerAngles(roll , pitch, yaw );
//	Eigen::Matrix3d Rwb = ypr2R(Eigen::Vector3d(yaw* 180/M_PI, pitch* 180/M_PI,roll* 180/M_PI));
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
	for (int i = 0; i < imudata.size(); ++i) {

			//force xy plane movement
//			imudata[i].imu_gyro[0] = 0;
//			imudata[i].imu_gyro[1] = 0;
//			imudata[i].imu_acc[2] = 9.81;
//			imudata[i].twb[2] = 0;

			imudata[i].rpy[2] = -(90 - imudata[i].rpy[2]);

//			double ax = imudata[i].imu_acc[1];
//			double ay = -imudata[i].imu_acc[0];
//
//			imudata[i].imu_acc[0] = ax;
//			imudata[i].imu_acc[1] = ay;
	}
	for (int i = 1; i < imudata.size(); ++i) {
		MotionData imupose = imudata[i];
		if(imupose.seq == 2861){
			cout<<"stop here"<<endl;
		}
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

		cout<<"acc_w:"<<acc_w<<endl;
		cout<<"v_w:"<<Vw<<endl;

		/// imu 动力学模型, consant turn rate, constant acceleration, CTRA
		Qwb = Qwb * dq;
		Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
		Vw = Vw + acc_w * dt;

		Eigen::Matrix3d Rwb(Qwb);
		Eigen::Vector3d euler_angles = R2ypr(Rwb);
//		Eigen::Vector3d euler_angles = Rwb.eulerAngles ( 2,1,0 );
//		euler_angles = euler_angles * 180/3.14;




		//　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
		save_points<<imupose.timestamp<<","
				<<euler_angles[2]<<","
			    <<euler_angles[1]<<","
			    <<euler_angles[0]<<","
			    <<imupose.rpy[0]<<","
			    <<imupose.rpy[1]<<","
			    <<imupose.rpy[2]<<","


				   <<Pwb(0)<<","
				   <<Pwb(1)<<","
				   <<Pwb(2)<<","
				   <<imupose.twb[0]<<","
				   <<imupose.twb[1]<<","
				   <<imupose.twb[2]<<","

				   <<imupose.imu_gyro[0]<<","
				   <<imupose.imu_gyro[1]<<","
				   <<imupose.imu_gyro[2]<<","

				   <<imupose.imu_acc[0]<<","
				   <<imupose.imu_acc[1]<<","
				   <<imupose.imu_acc[2]<<","
				   <<acc_w[0]<<","
				   <<acc_w[1]<<","
				   <<acc_w[2]<<","
				   <<Vw[0]<<","
				   <<Vw[1]<<","
				   <<Vw[2]<<","
				   <<imupose.seq<<","


				   <<std::endl;

		//print to screen
		std::cout<<imupose.timestamp<<","
				           <<"EUlER:"
						   <<euler_angles[2]<<","
						   <<euler_angles[1]<<","
						   <<euler_angles[0]<<"; "
						   <<imupose.rpy[0]<<","
						   <<imupose.rpy[1]<<","
						   <<imupose.rpy[2]<<"; "


						   <<"POSITION:"
						   <<Pwb(0)<<","
						   <<Pwb(1)<<","
						   <<Pwb(2)<<"; "
						   <<imupose.twb[0]<<","
						   <<imupose.twb[1]<<","
						   <<imupose.twb[2]<<","

						   <<"DATA:"
						   <<imupose.imu_gyro[0]<<","
						   <<imupose.imu_gyro[1]<<","
						   <<imupose.imu_gyro[2]<<"; "
						   <<imupose.imu_acc[0]<<","
						   <<imupose.imu_acc[1]<<","
						   <<imupose.imu_acc[2]<<","
						   <<"seq:"
						   <<imupose.seq<<","

						   <<std::endl;


	}

	std::cout<<"test　end"<<std::endl;



	return;
}

}
