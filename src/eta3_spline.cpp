#include <eta3_spline/eta3_spline.h>
#include <ros/ros.h>

/// constructor 1
Eta3Traj::Eta3Traj()
{
	eta_.resize(6);
	for (int i=0; i<6; i++)
	{
		eta_[i]=5; // the default etas are 5
	}
	d_t_ = 0.02; // the default delta t is 0.02s
	got_poses_ = false;
	ROS_INFO("Finished constructor.");
}
/// constructor 2
Eta3Traj::Eta3Traj(double d_t)
{
	eta_.resize(6);
	for (int i=0; i<6; i++)
	{
		eta_[i]=5; // the default etas are 5
	}
	d_t_ = d_t;
	got_poses_ = false;
	ROS_INFO("Finished constructor.");
}
/// constructor 3
Eta3Traj::Eta3Traj(std::vector<double> eta_v,double d_t)
{
	int n = eta_v.size();
	if(n!=6)
	{
		ROS_ERROR("Wrong etas! Using default etas...");
		eta_.resize(6);
		for (int i=0; i<6; i++)
		{
			eta_[i]=5; // the default etas are 5
		}
	}
	else 
		eta_ = eta_v;
	d_t_ = d_t;
	got_poses_ = false;
	ROS_INFO("Finished constructor.");
}

/// set poses to create trajectory
void Eta3Traj::setPoses(std::vector<geometry_msgs::PoseStamped> poses)
{
	got_poses_ = true;
	poses_nm_ = poses.size();
	poses_.resize(poses_nm_);
	poses_ = poses;
	ROS_INFO("Got poses with %d points.", poses_nm_);
}

/// get the interpolated trajectory
bool Eta3Traj::getTraj(std::vector<nav_msgs::Odometry> &traj,double &d_t)
{
	if (!got_poses_)
	{
		ROS_WARN("There is no poses inputted!");
		return false;
	}
	d_t = d_t_; // return the delta t
	bool built_traj = false;
	traj_.clear(); // reset the private container
	built_traj = buildTraj();
	if (!built_traj)
	{
		ROS_WARN("Failed to build the trajectory!");
		return false;
	}	
	traj.resize(traj_.size());
	traj = traj_;
	ROS_INFO("Got the trajectory with %d points.", traj.size());
	return true;
}

/// build the trajectory based on eta3-spline
bool Eta3Traj::buildTraj()
{
	double t; // the time for one path
	traj_.clear(); // reset the private container
	nav_msgs::Odometry pt;
	geometry_msgs::Quaternion q;
	std::vector<double> x_traj;
	std::vector<double> y_traj;
	std::vector<double> theta_traj;
	std::vector<double> velocity;
	std::vector<double> omega;
	bool got_t = false;
	for (int i=1; i<poses_nm_; i++)
	// Note i starts from 1
	// If we have n points from poses, there are (n-1) pathes, and the path index starts from 1 (not 0).
	// the points index starts from 0
	{
		// reset all containers
		x_traj.clear(); 
		y_traj.clear();
		theta_traj.clear();
		velocity.clear();
		omega.clear();
		// build trajectories and velocities
		t = calTime(i);
		got_t = false;
		got_t = buildXTraj(t,i,x_traj);
		if(!got_t)
		{
			ROS_WARN("Failed to build x trajectory!");
			return false;
		}
		got_t = false;
		got_t = buildYTraj(t,i,y_traj);
		if(!got_t)
		{
			ROS_WARN("Failed to build y trajectory!");
			return false;
		}
		got_t = false;
		got_t = buildTheta(t,i,theta_traj);
		if(!got_t)
		{
			ROS_WARN("Failed to build theta trajectory!");
			return false;
		}
		got_t = false;
		got_t = buildVel(t,i,velocity);
		if(!got_t)
		{
			ROS_WARN("Failed to build velocity!");
			return false;
		}
		got_t = false;
		got_t = buildOmega(t,i,omega);
		if(!got_t)
		{
			ROS_WARN("Failed to build omega!");
			return false;
		}
		// the interpolated points on one path should be consistent
		int x_n = x_traj.size();
		int y_n = y_traj.size();
		int t_n = theta_traj.size();
		int v_n = velocity.size();
		int o_n = omega.size();
		if (x_n == y_n && x_n==t_n && x_n==v_n && x_n==o_n)
		{
			for (int j=0; j<x_n; j++)
			{
				pt.pose.pose.position.x = x_traj[j];
				pt.pose.pose.position.y = y_traj[j];
				q = convertTheta2Quat(theta_traj[j]);
				pt.pose.pose.orientation = q;
				pt.twist.twist.linear.x = velocity[j];
				pt.twist.twist.angular.z = omega[j];
				traj_.push_back(pt);
			}
		}
		else
		{
			ROS_WARN("The points on the %d-th path is not consistent", i);
			return false;
		}

	}
}

/// calculate the time from point (index-1) to point index
double Eta3Traj::calTime(int index)
{

}
/// built the traj from point (index-1) to point index
bool Eta3Traj::buildXTraj(double t, int index, std::vector<double> & x_traj)
{

}
/// built the traj from point (index-1) to point index
bool Eta3Traj::buildYTraj(double t, int index, std::vector<double> & y_traj)
{

}
/// built the traj from point (index-1) to point index
bool Eta3Traj::buildTheta(double t, int index, std::vector<double> & theta_traj)
{

}
/// built the translational velocity from point (index-1) to point index
bool Eta3Traj::buildVel(double t, int index, std::vector<double> & veloity)
{

}
/// built the angular velocity from point (index-1) to point index
bool Eta3Traj::buildOmega(double t, int index, std::vector<double> & omega)
{

}

geometry_msgs::Quaternion Eta3Traj::convertTheta2Quat(double theta)
{

}