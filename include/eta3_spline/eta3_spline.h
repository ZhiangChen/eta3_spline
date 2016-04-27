/// Using eta3 spline to interpolate the trajectory for mobile robot
/// Zhiang Chen, 4/2016
/// Xinyu Li
/// CWRU EECS376, Prof.Wyatt Newman

#ifndef ETA3_SPLINE_H_
#define ETA3_SPLINE_H_
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>

#define max_vel 1.0
#define max_omega 1.0
#define Time_K 1.5

class Eta3Traj
{
public:
	Eta3Traj(); // default d_t; default etas
	Eta3Traj(double d_t);
	Eta3Traj(std::vector<double> eta_v, std::vector<double> kappa_v,double d_t);

	void setPoses(std::vector<geometry_msgs::PointStamped> poses);
	bool getTraj(std::vector<nav_msgs::Odometry> &traj,double &d_t);
	//bool brake(std::vector<nav_msgs::Odometry> traj, double t);
	//bool pause(std::vector<nav_msgs::Odometry> &traj, double t);
private:
	bool buildTraj();
	double calTime(int index);// start from 1
	bool buildXTraj(double t, int index, std::vector<double> & x_traj);
	bool buildYTraj(double t, int index, std::vector<double> & y_traj);
	bool buildTheta(double t, int index, std::vector<double> & theta_traj);
	bool buildVel(double t, int index, std::vector<double> & velocity);
	bool buildOmega(double t, int index, std::vector<double> & omega);
	geometry_msgs::Quaternion convertTheta2Quat(double theta);
	double convertTheta(double theta);
	double positiveTheta(double theta);

	std::vector<geometry_msgs::PointStamped> poses_;
	std::vector<double> eta_;
	double k_start_;
	double k_start_dot_;
	double k_end_;
	double k_end_dot_;
	std::vector<nav_msgs::Odometry> traj_;
	double d_t_;
	bool got_poses_;
	int poses_nm_;
	std::vector<double> alpha_;
	std::vector<double> beta_;

	geometry_msgs::PointStamped start_pose_;
	geometry_msgs::PointStamped end_pose_;

};

#endif

