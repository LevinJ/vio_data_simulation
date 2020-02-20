//
// Created by hyj on 17-6-22.
//

#include <fstream>
#include <sys/stat.h>
#include "../src/imu.h"
#include "../src/utilities.h"

using Point = Eigen::Vector4d;
using Points = std::vector<Point, Eigen::aligned_allocator<Point> >;
using Line = std::pair<Eigen::Vector4d, Eigen::Vector4d>;
using Lines = std::vector<Line, Eigen::aligned_allocator<Line> >;


extern "C" {

void propagate(char * src, char * dist, double x, double y, double z,
		double roll, double pitch, double yaw, double vx, double vy, double vz, double start_time_stamp);
}
int main(){



//    double x =  29.4051809009;
//    double y= 10.690242751 ;
//    double z=  -0.0711993342265;
//    double roll = 0 ;
//    double pitch = 0;
//    double yaw =  -(90 - 24.9017483279) /180 * 3.14;
//    double vx = 2.596471982;
//    double vy= 1.204643166;
//    double vz= -0.098029840738 ;

	//first straight line
//   double x =  0.345888043288141;
//   double y= 0.139293533284217 ;
//   double z=  -0.044415683485568;
//   double roll = 0 ;
//   double pitch = 0;
//   double yaw =  -(90 - 24.9017483279) /180 * 3.14;
//   double vx = 0.334116742445669;
//   double vy= 0.135681490839337;
//   double vz= 0.24317090642734 ;

	//correct roll and pitch
//  double x =  0;
//  double y= 0 ;
//  double z=  0;
//  double roll = 0.644819 /180 * 3.14;
//  double pitch = -1.39424/180 * 3.14;
//  double yaw =  -(90 - 24.9017483279) /180 * 3.14;
//  double vx = 0;
//  double vy= 0;
//  double vz= 0 ;

 //start with 4408
//  double x =  2.73761787742842;
//	double y= 1.07021573698148 ;
//	double z=  -0.028857103548944;
//	double roll = 0.350598 /180 * 3.14;
//	double pitch = -1.214435 /180 * 3.14;
//	double yaw =  -(90 - 24.9017483279) /180 * 3.14;
//	double vx = 0.005078;
//	double vy= 0.033214;
//	double vz= -0.019082 ;
//	double start_time_stamp = 1576825203.865939;

	//incorrect roll and pitch
//  double x =  0;
//    double y= 0 ;
//    double z=  0;
//    double roll = 0;
//    double pitch = 0;
//    double yaw =  -(90 - 24.9017483279) /180 * 3.14;
//    double vx = 0;
//    double vy= 0;
//    double vz= 0 ;

//	double x =  -1.619314;
//	double y= 2.743600 ;
//	double z=  -0.071857;
//	double yaw =  -(90 - 24.9017483279) /180 * 3.14;
//	double pitch = -2.328406 /180 * 3.14;
//	double roll = 0.456945  /180 * 3.14;
//
//
//	double vx = -0.643986;
//	double vy= 1.830101;
//	double vz= -0.020833 ;
//	double start_time_stamp = 1576825203.865939;

//	double xyz[3]={2.73761787742842,	1.07021573698148,	-0.028857103548944};
//	double vxyz[3]={1.86328634154051,	0.640195650048556,	-0.029704933986068};

	double xyz[3]={0,	0,	0};
	double vxyz[3]={1.54153, 1.178, -0.020833};

	double ypr[3]={-1.987571, -2.328406, 0.456945};

	double x =  xyz[0];
	double y= xyz[1] ;
	double z=  xyz[2];

	double yaw =  -(90 - 17.9017483279) /180 * 3.14;
	double pitch = ypr[1] /180 * 3.14;
	double roll = ypr[2] /180 * 3.14;


	double vx = vxyz[0];
	double vy= vxyz[1];
	double vz= vxyz[2] ;
	double start_time_stamp = 1576825203.865939;

    propagate("/home/levin/workspace/vio_course/vio_data_simulation/python_tool/slam/temp/1220_41/1220_41_imudata.csv",
    		"/home/levin/workspace/vio_course/vio_data_simulation/python_tool/slam/temp/1220_41/1220_41_imudata_int.csv",
    		x, y, z,
    		roll, pitch, yaw,
			vx, vy, vz,start_time_stamp);



    return 0;
}
