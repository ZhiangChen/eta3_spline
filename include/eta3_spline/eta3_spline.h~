/// Using eta3 spline to interpolate the trajectory for mobile robot
/// Zhiang Chen, Xinyu Li; 4/2016
/// CWRU EECS376, Prof.Wyatt Newman

#ifndef ETA3_SPLINE_H_
#define ETA3_SPLINE_H_
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#define vel 1.0
#define omega 1.0

class Eta3Traj
{
public:
	Eta3Traj(); // default d_t; default etas
	Eta3Traj(double d_t);
	Eta3Traj(std::vector<double> eta_v,double d_t);

	void setPoses(std::vector<geometry_msgs::PoseStamped> poses);
	bool getTraj(std::vector<nav_msgs::Odometry> &traj,double &d_t);
	//bool brake(std::vector<nav_msgs::Odometry> traj, double t);
	//bool pause(std::vector<nav_msgs::Odometry> &traj, double t);
private:
	bool buildTraj();
	double calTime(int index);// start from 1
	bool buildXTraj();
	bool buildYTraj();
	bool buildTheta();
	bool buildVel();
	bool buildOmega();

	std::vector<geometry_msgs::PoseStamped> poses_;
	std::vector<double> eta_v_;
	std::vector<nav_msgs::Odometry> traj_;
	double d_t_;
	bool got_poses_;
	int poses_nm_;
};

#endif

