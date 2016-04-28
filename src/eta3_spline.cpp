/// Using eta3 spline to interpolate the trajectory for mobile robot
/// Zhiang Chen, 4/2016

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
	k_start_ = 0;
	k_start_dot_ = 0;
	k_end_ = 0;
	k_end_dot_ = 0;
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
	k_start_ = 0;
	k_start_dot_ = 0;
	k_end_ = 0;
	k_end_dot_ = 0;
	got_poses_ = false;
	ROS_INFO("Finished constructor.");
}
/// constructor 3
Eta3Traj::Eta3Traj(std::vector<double> eta_v, std::vector<double> kappa_v, double d_t)
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
	n = kappa_v.size();
	if(n!=4)
	{
		ROS_ERROR("Wrong kappas! Using default kappas");
		k_start_ = 0;
		k_start_dot_ = 0;
		k_end_ = 0;
		k_end_dot_ = 0;	
	}
	else
	{
		k_start_ = kappa_v[0];
		k_start_dot_ = kappa_v[1];
		k_end_ = kappa_v[2];
		k_end_dot_ = kappa_v[3];
	}
	d_t_ = d_t;
	got_poses_ = false;
	ROS_INFO("Finished constructor.");
}

/// set poses to create trajectory
void Eta3Traj::setPoses(std::vector<geometry_msgs::PointStamped> poses)
{
	got_poses_ = true;
	poses_nm_ = poses.size();
	poses_.resize(poses_nm_);
	poses_ = poses;
	ROS_INFO("Got poses with %d points.", poses_nm_);
}

/// set etas
void Eta3Traj::setEta(std::vector<double> eta_v)
{
	int n = eta_v.size();
	if(n!=6)
	{
		ROS_ERROR("Wrong etas! Using default etas...");
		return;
	}
	else 
		eta_ = eta_v;
	ROS_INFO("Set eta done.");
}

/// set kappas
void Eta3Traj::setKappa(std::vector<double> kappa_v)
{
	int n = kappa_v.size();
	if(n!=4)
	{
		ROS_ERROR("Wrong kappas! Using default kappas");
		return;
	}
	else
	{
		k_start_ = kappa_v[0];
		k_start_dot_ = kappa_v[1];
		k_end_ = kappa_v[2];
		k_end_dot_ = kappa_v[3];
	}	
	ROS_INFO("Set kappa done.");
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
	int n = traj.size();
	ROS_INFO("Got the trajectory with %d points.", n);
	return true;
}

/// start running trajectory
bool Eta3Traj::runTraj(std::vector<nav_msgs::Odometry> traj)
{
	ROS_INFO("Running trajectory. To brake: rostopic pub /brake std_msgs/Bool true ");
	desired_state_publisher_ = nh_.advertise<nav_msgs::Odometry>("/desState", 1,true);
	int n = traj.size();
	current_i_ = 0;
	got_brake_ = false;
	brake_subscriber_ = nh_.subscribe("/brake",1,&Eta3Traj::brakeCallback,this); 
	while (ros::ok() && current_i_<n && !got_brake_) 
    {
    	desired_state_publisher_.publish(traj[current_i_++]);
    	ros::spinOnce();
    	ros::Duration(d_t_).sleep();
    }
    if (current_i_ < n)
    	return false;
    else
    	ROS_INFO("Reached the goal.");
	return true;
}

/// start running trajectory, and publishing the desired pose at the same time
bool Eta3Traj::runTraj2(std::vector<nav_msgs::Odometry> traj)
{
	ROS_INFO("Running trajectory. To brake: rostopic pub /brake std_msgs/Bool true ");
	desired_state_publisher_ = nh_.advertise<nav_msgs::Odometry>("/desState", 1,true);
	x_publisher_ = nh_.advertise<std_msgs::Float32>("/des_x_position", 1, true);
    y_publisher_ = nh_.advertise<std_msgs::Float32>("/des_y_position", 1, true);
    theta_publisher_ = nh_.advertise<std_msgs::Float32>("/des_theta_position", 1, true);
    v_publisher_ = nh_.advertise<std_msgs::Float32>("/des_velocity", 1, true);
    ROS_INFO("The topic names are: /des_x_position, /des_y_position, /des_theta_position, /des_velocity");
	int n = traj.size();
	current_i_ = 0;
	got_brake_ = false;
	std_msgs::Float32 x,y,theta,v;
	brake_subscriber_ = nh_.subscribe("/brake",1,&Eta3Traj::brakeCallback,this); 
	while (ros::ok() && current_i_<n && !got_brake_) 
    {
    	x.data = traj[current_i_].pose.pose.position.x;
    	y.data = traj[current_i_].pose.pose.position.y;
    	v.data = traj[current_i_].twist.twist.linear.x;
    	theta.data = convertPlanarQuat2Psi(traj[current_i_].pose.pose.orientation);
    	x_publisher_.publish(x);
    	y_publisher_.publish(y);
    	theta_publisher_.publish(theta);
    	v_publisher_.publish(v);
    	desired_state_publisher_.publish(traj[current_i_++]);
    	ros::spinOnce();
    	ros::Duration(d_t_).sleep();
    }
    if (current_i_ < n)
    	return false;
    else
    	ROS_INFO("Reached the goal.");
	return true;
}

void Eta3Traj::brakeCallback(const std_msgs::Bool brake)
{
	if (brake.data == true)
	{
		ROS_WARN("Brake!");
		got_brake_ = true;
	}
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
		buildVel(t,i,theta_traj,velocity);
		buildOmega(t,i,omega);
		// the interpolated points on one path should be consistent
		int x_n = x_traj.size();
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
	return true;
}

/// calculate the time from point (index-1) to point index
double Eta3Traj::calTime(int index)
{
	double d_x;
	double d_y;
	double d_theta;
	start_pose_ = poses_[index-1];
	end_pose_ = poses_[index];
	d_x = start_pose_.point.x - end_pose_.point.x;
	d_y = start_pose_.point.y - end_pose_.point.y;
	d_theta = start_pose_.point.z - end_pose_.point.z;

	double t1 = sqrt(d_x*d_x + d_y*d_y)/max_vel;
	double t2 = d_theta/max_omega;

	return (t1>t2?t1*Time_K:t2*Time_K);
}
/// built the traj from point (index-1) to point index
bool Eta3Traj::buildXTraj(double t, int index, std::vector<double> & x_traj)
{
	double x_A = start_pose_.point.x;
	double x_B = end_pose_.point.x;
	double theta_A = start_pose_.point.z;
	double theta_B = end_pose_.point.z;
	theta_A = convertTheta(theta_A);
	theta_B = convertTheta(theta_B);
	alpha_.resize(8);

	alpha_[0] = x_A;
	alpha_[1] = eta_[0]*cos(theta_A);
	alpha_[2] = 0.5*eta_[2]*cos(theta_A)-0.5*eta_[0]*eta_[0]*k_start_*sin(theta_A);
	alpha_[3] = 1.0/6.0*eta_[4]*cos(theta_A)-1.0/6.0*(eta_[0]*eta_[0]*eta_[0]*k_start_dot_+3*eta_[0]*eta_[2]*k_start_)*sin(theta_A);
	alpha_[4] = 35*(x_B-x_A)-(20*eta_[0]+5*eta_[2]+2.0/3.0*eta_[4])*cos(theta_A)+(5*eta_[0]*eta_[0]*k_start_
				+2.0/3.0*eta_[0]*eta_[0]*eta_[0]*k_start_dot_+2*eta_[0]*eta_[2]*k_start_)*sin(theta_A)
	            -(15*eta_[1]-2.5*eta_[3]+1.0/6.0*eta_[5])*cos(theta_B)-(2.5*eta_[1]*eta_[1]*k_end_-1.0/6.0*eta_[1]*eta_[1]*eta_[1]*k_end_dot_
	            -0.5*eta_[1]*eta_[3]*k_end_)*sin(theta_B);
	alpha_[5] = -84*(x_B-x_A)+(45*eta_[0]+10*eta_[2]+eta_[4])*cos(theta_A)-(10*eta_[0]*eta_[0]*k_start_
				+eta_[0]*eta_[0]*eta_[0]*k_start_dot_+3*eta_[0]*eta_[2]*k_start_)*sin(theta_A)
				+(39*eta_[1]-7*eta_[3]+0.5*eta_[5])*cos(theta_B)+(7*eta_[1]*eta_[1]*k_end_-1.0/2.0*eta_[1]*eta_[1]*eta_[1]*k_end_dot_
				-1.5*eta_[1]*eta_[3]*k_end_)*sin(theta_B);
	alpha_[6] = 70*(x_B-x_A)-(36*eta_[0]+7.5*eta_[2]+2.0/3.0*eta_[4])*cos(theta_A)
				+(7.5*eta_[0]*eta_[0]*k_start_+2.0/3.0*eta_[0]*eta_[0]*eta_[0]*k_start_dot_
				+2*eta_[0]*eta_[2]*k_start_)*sin(theta_A)-(34*eta_[1]-6.5*eta_[3]+0.5*eta_[5])*cos(theta_B)
				-(6.5*eta_[1]*eta_[1]*k_end_-1.0/2.0*eta_[1]*eta_[1]*eta_[1]*k_end_dot_-1.5*eta_[1]*eta_[3]*k_end_)*sin(theta_B);
	alpha_[7] = -20*(x_B-x_A)+(10*eta_[0]+2*eta_[2]+1.0/6.0*eta_[4])*cos(theta_A)
	            -(2*eta_[0]*eta_[0]*eta_[0]*k_start_+1.0/6.0*eta_[0]*eta_[0]*eta_[0]*k_start_dot_+1.0/2.0*eta_[0]*eta_[2]*k_start_)*sin(theta_A)
	            +(10*eta_[1]-2*eta_[3]+1.0/6.0*eta_[5])*cos(theta_B)+(2*eta_[1]*eta_[1]*k_end_
	            -1.0/6.0*eta_[1]*eta_[1]*eta_[1]*k_end_dot_-0.5*eta_[1]*eta_[3]*k_end_)*sin(theta_B);

	int nm = t/d_t_;
	double d_u = 1.0/nm;
	double u=0;
	x_traj.resize(nm);
	for (int j=0; j<nm; j++)
	{
		x_traj[j] = alpha_[0] + alpha_[1]*u + alpha_[2]*u*u + alpha_[3]*u*u*u + alpha_[4]*u*u*u*u 
					+alpha_[5]*u*u*u*u*u + alpha_[6]*u*u*u*u*u*u + alpha_[7]*u*u*u*u*u*u*u;
		u += d_u;
	}

	return true;
}
/// built the traj from point (index-1) to point index
bool Eta3Traj::buildYTraj(double t, int index, std::vector<double> & y_traj)
{
	double y_A = start_pose_.point.y;
	double y_B = end_pose_.point.y;
	double theta_A = start_pose_.point.z;
	double theta_B = end_pose_.point.z;
	theta_A = convertTheta(theta_A);
	theta_B = convertTheta(theta_B);
	beta_.resize(8);

	beta_[0] = y_A;
	beta_[1] = eta_[0]*sin(theta_A);
	beta_[2] = 0.5*eta_[2]*sin(theta_A)+0.5*eta_[0]*eta_[0]*k_start_*cos(theta_A);
	beta_[3] = 1.0/6.0*eta_[4]*sin(theta_A)+1.0/6.0*(eta_[0]*eta_[0]*eta_[0]*k_start_dot_
				+3*eta_[0]*eta_[2]*k_start_)*cos(theta_A);
	beta_[4] = 35*(y_B-y_A)-(20*eta_[0]+5*eta_[2]+2.0/3.0*eta_[4])*sin(theta_A)
	          -(5*eta_[0]*eta_[0]*k_start_+2.0/3.0*eta_[0]*eta_[0]*eta_[0]*k_start_dot_
	          	+2*eta_[0]*eta_[2]*k_start_)*cos(theta_A)-(15*eta_[1]-2.5*eta_[3]+1.0/6.0*eta_[5])*sin(theta_B)
	          +(2.5*eta_[1]*eta_[1]*k_end_-1.0/6.0*eta_[1]*eta_[1]*eta_[1]*k_end_dot_-0.5*eta_[1]*eta_[3]*k_end_)*cos(theta_B);
	beta_[5] = -84*(y_B-y_A)+(45*eta_[0]+10*eta_[2]+eta_[4])*sin(theta_A)
				+(10*eta_[0]*eta_[0]*k_start_+eta_[0]*eta_[0]*eta_[0]*k_start_dot_+
				3*eta_[0]*eta_[2]*k_start_)*cos(theta_A)+(39*eta_[1]-7*eta_[3]+0.5*eta_[5])*sin(theta_B)
				-(7*eta_[1]*eta_[1]*k_end_-1.0/2.0*eta_[1]*eta_[1]*eta_[1]*k_end_dot_-1.5*eta_[1]*eta_[3]*k_end_)*cos(theta_B);
	beta_[6] = 70*(y_B-y_A)-(36*eta_[0]+7.5*eta_[2]+2.0/3.0*eta_[4])*sin(theta_A)
	           -(7.5*eta_[0]*eta_[0]*k_start_+2.0/3.0*eta_[0]*eta_[0]*eta_[0]*k_start_dot_+2*eta_[0]*eta_[2]*k_start_)*cos(theta_A)
	           -(34*eta_[1]-6.5*eta_[3]+0.5*eta_[5])*sin(theta_B)+(6.5*eta_[1]*eta_[1]*k_end_-1.0/2.0*eta_[1]*eta_[1]*eta_[1]*k_end_dot_
	           	-1.5*eta_[1]*eta_[3]*k_end_)*cos(theta_B);
	beta_[7] = -20*(y_B-y_A)+(10*eta_[0]+2*eta_[2]+1.0/6.0*eta_[4])*sin(theta_A)+(2*eta_[0]*eta_[0]*k_start_
				+1.0/6.0*eta_[0]*eta_[0]*eta_[0]*k_start_dot_+1.0/2.0*eta_[0]*eta_[2]*k_start_)*cos(theta_A)
				+(10*eta_[1]-2*eta_[3]+1.0/6.0*eta_[5])*sin(theta_B)-(2*eta_[1]*eta_[1]*k_end_
				-1.0/6.0*eta_[1]*eta_[1]*eta_[1]*k_end_dot_-0.5*eta_[1]*eta_[3]*k_end_)*cos(theta_B);

	int nm = t/d_t_;
	double d_u = 1.0/nm;
	double u=0;
	y_traj.resize(nm);
	for (int j=0; j<nm; j++)
	{
		y_traj[j] = beta_[0] + beta_[1]*u + beta_[2]*u*u + beta_[3]*u*u*u + beta_[4]*u*u*u*u 
					+beta_[5]*u*u*u*u*u + beta_[6]*u*u*u*u*u*u + beta_[7]*u*u*u*u*u*u*u;
		u += d_u;
	}
	return true;
}
/// built the traj from point (index-1) to point index
bool Eta3Traj::buildTheta(double t, int index, std::vector<double> & theta_traj)
{
	int nm = t/d_t_;
	double d_u = 1.0/nm;
	double u=0;
	theta_traj.resize(nm);
	double dy_du;
	double dx_du;
	for (int j=0; j<nm; j++)
	{
		dx_du = alpha_[1] + 2*alpha_[2]*u + 3*alpha_[3]*u*u + 4*alpha_[4]*u*u*u
				+5*alpha_[5]*u*u*u*u + 6*alpha_[6]*u*u*u*u*u + 7*alpha_[7]*u*u*u*u*u*u;
		dy_du = beta_[1] + 2*beta_[2]*u + 3*beta_[3]*u*u + 4*beta_[4]*u*u*u
				+5*beta_[5]*u*u*u*u + 6*beta_[6]*u*u*u*u*u + 7*beta_[7]*u*u*u*u*u*u;
		theta_traj[j] = atan2(dy_du,dx_du);
		u += d_u;
	}
	return true;	
}
/// built the translational velocity from point (index-1) to point index
bool Eta3Traj::buildVel(double t, int index, std::vector<double> theta_traj, std::vector<double> & velocity)
{
	int nm = t/d_t_;
	double d_u = 1.0/nm;
	double u=0;
	velocity.resize(nm);
	double dx_du;
	for (int j=0; j<nm; j++)
	{
		//velocity[j] = 0; // when using steering algorithm
		dx_du = alpha_[1] + 2*alpha_[2]*u + 3*alpha_[3]*u*u + 4*alpha_[4]*u*u*u
				+5*alpha_[5]*u*u*u*u + 6*alpha_[6]*u*u*u*u*u + 7*alpha_[7]*u*u*u*u*u*u;

		velocity[j] = dx_du/t/cos(theta_traj[j]);

		u += d_u;
	}
	return true;
}
/// built the angular velocity from point (index-1) to point index
bool Eta3Traj::buildOmega(double t, int index, std::vector<double> & omega)
{
	int nm = t/d_t_;
	omega.resize(nm);
	for (int j=0; j<nm; j++)
	{
		omega[j] = 0; // when using steering algorithm
		// Or omega = cos(theta)^2*d(dy_dx)/dt
	}
	return true;
}

geometry_msgs::Quaternion Eta3Traj::convertTheta2Quat(double theta)
{
	// The rotation axel is (0,0,1)
	geometry_msgs::Quaternion q;
	q.w = cos(theta/2.0);
	q.x = 0;
	q.y = 0;
	q.z = sin(theta/2.0);
	return q;
}

double Eta3Traj::convertTheta(double theta)
{
	double c_theta;
	c_theta = positiveTheta(theta);
	while(c_theta>2*M_PI)
	{
		c_theta -= 2*M_PI;
	}
	return c_theta;
}

double Eta3Traj::positiveTheta(double dang) {
    while (dang<0)
    {
    	dang = dang+2*M_PI;
    }
    return dang;
}

double Eta3Traj::convertPlanarQuat2Psi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double psi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return psi;
}