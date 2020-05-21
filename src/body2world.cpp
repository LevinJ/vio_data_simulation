#include <iostream>
#include <string>
#include <iomanip>
#include <cmath>
using namespace std;

#include <iostream>
#include <eigen3/Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

template <typename Derived>
     Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t y = ypr(0) / 180.0 * M_PI;
        Scalar_t p = ypr(1) / 180.0 * M_PI;
        Scalar_t r = ypr(2) / 180.0 * M_PI;

        Eigen::Matrix<Scalar_t, 3, 3> Rz;
        Rz << cos(y), -sin(y), 0,
            sin(y), cos(y), 0,
            0, 0, 1;

        Eigen::Matrix<Scalar_t, 3, 3> Ry;
        Ry << cos(p), 0., sin(p),
            0., 1., 0.,
            -sin(p), 0., cos(p);

        Eigen::Matrix<Scalar_t, 3, 3> Rx;
        Rx << 1., 0., 0.,
            0., cos(r), -sin(r),
            0., sin(r), cos(r);

        return Rz * Ry * Rx;
    }

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
Matrix3d getvb2gb(){
	//compute the transform from base_link in VIO to base_link in UTM
	Vector3d ypr;
	ypr<<-90, 0,0;
	static MatrixXd R;
	static bool first_time = true;
	if(first_time){
		first_time = false;
		R = ypr2R(ypr);
	}
	return R;
}

class PoseInfo{
public:
	PoseInfo(string _name):pose(7), name(_name){
		shift_base_link = false;
	}
	string name;
	Vector3d P;
	Matrix3d R;
	Vector3d ypr;
	VectorXd pose;
	bool shift_base_link;
};
void convert(PoseInfo & poseinfo){
	VectorXd &pose = poseinfo.pose;
	Vector3d &P = poseinfo.P;
	Matrix3d &R = poseinfo.R;
	Vector3d &ypr =  poseinfo.ypr;
	cout<<poseinfo.name<<",";
	P = pose.segment(0, 3);
	double q_x, q_y, q_z, q_w;
	q_x = pose[3];
	q_y = pose[4];
	q_z = pose[5];
	q_w = pose[6];
	Eigen::Quaterniond q(q_w,q_x, q_y, q_z);
	if(poseinfo.shift_base_link){
		q = q * Eigen::Quaterniond(getvb2gb());
	}

	R = q;
	ypr = R2ypr(R);
//	cout<<"pose="<<pose.transpose();
	cout<<",position="<<P.transpose();
	cout<<",ypr="<<ypr.transpose();
	cout<<endl<<", R="<<R<<endl;

}

void getVG(PoseInfo & vb, PoseInfo & gb, PoseInfo & vg){

	Matrix3d Rvg = vb.R * gb.R.transpose();
	Vector3d tvg = -Rvg * gb.P + vb.P;

//	Rvg = Rvg.transpose().eval();
//	tvg = -Rvg * tvg;

	Eigen::Quaterniond q(Rvg);
	vg.pose.segment(0, 3) = tvg;
	vg.pose[3] = q.x();
	vg.pose[4] = q.y();
	vg.pose[5] = q.z();
	vg.pose[6] = q.w();

}

int main()
{
	cout.setf(ios::fixed);
	std::cout<<setprecision(6);

	Eigen::Quaterniond q(0.977871201916, 0.00176938361239,  -0.0107957019332,  0.208921599085);
	Matrix3d R;
	R = q;
	Vector3d ypr;
	ypr = R2ypr(R);
	cout<<ypr<<endl;




/*	Vector3d ypr;
	ypr<<-65, 0,0;
	auto R = ypr2R(ypr);
	Eigen::Quaterniond q(R);
//	cout<<q;


	PoseInfo t0_v("t0_v");
	t0_v.pose<<0, 5, 0, 0, 0, 0, 1;
	convert(t0_v);

	PoseInfo t0_g("t0_g");
	t0_g.pose<<4.532, 2.11, 0, q.x(), q.y(), q.z(),q.w();
	convert(t0_g);
//
	PoseInfo t0_vg("t0_vg");
	getVG(t0_v, t0_g, t0_vg);
	convert(t0_vg);
	*/

//	//points for t1
//	PoseInfo t1_v("t1_v");
//	t1_v.pose<<0, 9.0123400, 0, -0.0015806, 0.0036900, 0.0059892, 0.9999740;
//	convert(t1_v);
//
//	PoseInfo t1_g("t1_g");
//	t1_g.pose<<8.4, 3.548564, 0, -0.000272, -0.007695, 0.215388, 0.976498;
//	convert(t1_g);
//
//	PoseInfo t1_vg("t1_vg");
//	getVG(t1_v, t1_g, t1_vg);
//	convert(t1_vg);

//	PoseInfo t0_v("t0_v");
//	t0_v.pose<<-0.0369801, 4.9809915, -0.1591650, 0.0004093, 0.0019082, 0.0025026, 0.9999950;
//	convert(t0_v);
//
//	PoseInfo t0_g("t0_g");
//	t0_g.shift_base_link = true;
//	t0_g.pose<<4.650712, 1.999425, 0.055847, 0.001610, -0.008749, 0.211701, 0.977294;
//	convert(t0_g);
//
//	PoseInfo t0_vg("t0_vg");
//	getVG(t0_v, t0_g, t0_vg);
//	convert(t0_vg);
//
//	//points for t1
//	PoseInfo t1_v("t1_v");
//	t1_v.pose<<-0.1029960, 9.0123400, -0.2488936, -0.0015806, 0.0036900, 0.0059892, 0.9999740;
//	convert(t1_v);
//
//	PoseInfo t1_g("t1_g");
//	t1_g.shift_base_link = true;
//	t1_g.pose<<8.354912, 3.548564, 0.035288, -0.000272, -0.007695, 0.215388, 0.976498;
//	convert(t1_g);
//
//	PoseInfo t1_vg("t1_vg");
//	getVG(t1_v, t1_g, t1_vg);
//	convert(t1_vg);
//
//	auto R = t0_vg.R * t1_g.R;
//	auto t = t0_vg.R * t1_g.P + t0_vg.P;
//	Eigen::Quaterniond q(R);
//	PoseInfo res("res");
//	res.pose<<t[0],t[1],t[2],q.x(),q.y(),q.z(),q.w();
//	convert(res);



}
