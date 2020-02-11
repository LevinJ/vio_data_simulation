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
		double roll, double pitch, double yaw, double vx, double vy, double vz);
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
  double x =  2.73761787742842;
    double y= 1.07021573698148 ;
    double z=  -0.028857103548944;
    double roll = 0.350598 /180 * 3.14;
    double pitch = -1.214435 /180 * 3.14;
    double yaw =  -(90 - 24.9017483279) /180 * 3.14;
    double vx = 1.86328634154051;
    double vy= 0.640195650048556;
    double vz= -0.029704933986068 ;

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

    propagate("/home/levin/workspace/vio_course/vio_data_simulation/python_tool/slam/temp/1220_41/1220_41_imudata.csv",
    		"/home/levin/workspace/vio_course/vio_data_simulation/python_tool/slam/temp/1220_41/1220_41_imudata_int.csv",
    		x, y, z,
    		roll, pitch, yaw,
			vx, vy, vz);



    return 0;
}
