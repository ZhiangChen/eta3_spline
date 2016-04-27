/// Using eta3 spline to interpolate the trajectory for mobile robot
/// Zhiang Chen, 4/2016
/// CWRU EECS376, Prof.Wyatt Newman

#ifndef ETA3_SPLINE_H_
#define ETA3_SPLINE_H_
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

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
	bool runTraj();
	bool runTraj2();

private:
	bool buildTraj();
	double calTime(int index);// start from 1
	bool buildXTraj(double t, int index, std::vector<double> & x_traj);
	bool buildYTraj(double t, int index, std::vector<double> & y_traj);
	bool buildTheta(double t, int index, std::vector<double> & theta_traj);
	bool buildVel(double t, int index, std::vector<double> theta_traj, std::vector<double> & velocity);
	bool buildOmega(double t, int index, std::vector<double> & omega);
	geometry_msgs::Quaternion convertTheta2Quat(double theta);
	double convertPlanarQuat2Psi(geometry_msgs::Quaternion quaternion);
	double convertTheta(double theta);
	double positiveTheta(double theta);
	void brakeCallback(const std_msgs::Bool brake);

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
	int current_i_;
	ros::NodeHandle nh_;
	ros::Publisher desired_state_publisher_;
	bool got_brake_;

	ros::Subscriber brake_subscriber_;
	ros::Publisher x_publisher_; 
    ros::Publisher y_publisher_;
    ros::Publisher theta_publisher_;
    ros::Publisher v_publisher_;
};

#endif

