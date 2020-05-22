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
using Eigen::Matrix4d;




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
Matrix4d constructT(Matrix3d &R, Vector3d &t){
	Matrix4d T( Matrix4d::Zero());
	T.block(0, 0, 3, 3).noalias() = R;
	T.block(0, 3, 3, 1).noalias() = t;
	T(3,3) = 1;
	return T;
}
Matrix4d getvb2gbT(){
	//compute the transform from base_link in VIO to base_link in UTM
	auto R = getvb2gb();
	Vector3d t(Vector3d::Zero());
//	t<<0.29,-0.31,-0.84;
	t<<-0.31,-0.29,-0.84;
	auto T = constructT(R, t);
	return T;
}

Matrix4d getgb2vbT(){
	Vector3d ypr;
	ypr<<90, 0,0;
	static Matrix3d R;
	static bool first_time = true;
	if(first_time){
		first_time = false;
		R = ypr2R(ypr);
	}
	Vector3d t(Vector3d::Zero());
	t<<-0.29,0.31,0.84;
	Matrix4d T = constructT(R, t);
	return T;
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
	Matrix4d T;
	bool shift_base_link;
public:
	void convert(){
		cout<<name<<",";
		P = pose.segment(0, 3);
		double q_x, q_y, q_z, q_w;
		q_x = pose[3];
		q_y = pose[4];
		q_z = pose[5];
		q_w = pose[6];
		Eigen::Quaterniond q(q_w,q_x, q_y, q_z);
		if(shift_base_link){
			q = q * Eigen::Quaterniond(getvb2gb());
		}

		R = q;
		ypr = R2ypr(R);
		T = constructT(R, P);
	//	cout<<"pose="<<pose.transpose();
		cout<<",position="<<P.transpose();
		cout<<",ypr="<<ypr.transpose();
		cout<<",\nT="<<T;
		cout<<endl<<", R="<<R<<endl;
	}
};

void convertfromT(string name , Matrix4d &_T){
	auto T = _T;
	auto R = T.block(0,0,3,3);
	auto P = T.block(0,3,3,1);
	auto ypr = R2ypr(R);
	cout<<name<<": position = "<<P.transpose()<<",ypr="<<ypr.transpose()<<endl;
	cout<<name<<": T="<<T<<endl;
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


void test_acc_est(){
	//initialize estimated relative pose
	PoseInfo Toc("Toc");
	Toc.pose<<1.472596,3.737560,-0.089476,0.003098,0.009812,0.006837,0.999924;
	Toc.convert();
	Matrix4d est_rel_pose = Toc.T;
	convertfromT("est_rel_pose", est_rel_pose);

	//initialize ground truth pose
	PoseInfo TGo_gb("TGo_gb");
	TGo_gb.pose<<331383.430427,	3470499.08365,	14.7564034945,-0.00144698227995, -0.0117098317469,	0.221215935085,	0.975153473125;
	TGo_gb.convert();

	PoseInfo TGc_gb("TGc_gb");
	TGc_gb.pose<<331387.457553,	3470499.17539,	14.6627616394,-0.000116566808233,-0.000860592516973,	0.230311272308,	0.973116623864;
	TGc_gb.convert();

	Matrix4d Tgbvb = getgb2vbT().inverse();
	convertfromT("Tgbvb", Tgbvb);

	Matrix4d gt_rel_pose = (TGo_gb.T * Tgbvb).inverse() * (TGc_gb.T * Tgbvb);

	convertfromT("gt_rel_pose", gt_rel_pose);

	Matrix4d err = gt_rel_pose.inverse() * est_rel_pose;

	convertfromT("err", err);
	cout<<"distance = "<<gt_rel_pose.block(0,3,3,1).norm()<<endl;


}

void test_basic_transform(){
	Eigen::Quaterniond q(0.975153473125,-0.00144698227995,    -0.0117098317469,   0.221215935085);
	//	cout<<"q="<<q;
	Matrix3d R(q);
	cout<<"R"<<R<<endl;

	Eigen::Quaterniond q2(R);
	cout<<"q="<<q2.x()<<","<<q2.y()<<","<<q2.z()<<","<<q2.w()<<endl;

	Vector3d ypr;
	ypr = R2ypr(R);
	cout<<"ypr="<<ypr<<endl;

	Matrix3d R2;
	R2 = ypr2R(ypr);
	cout<<"R="<<R<<endl;
}

int main()
{

	cout.setf(ios::fixed);
	std::cout<<setprecision(6);







	test_acc_est();

//	Eigen::Quaterniond q(0.977871201916, 0.00176938361239,  -0.0107957019332,  0.208921599085);
//	Matrix3d R;
//	R = q;
//	Vector3d ypr;
//	ypr = R2ypr(R);
//	cout<<ypr<<endl;




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
