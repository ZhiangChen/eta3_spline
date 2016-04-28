/// The test main file for library Eta3Traj
/// Zhiang Chen, 4/2016

#include<ros/ros.h> 
#include <eta3_spline/eta3_spline.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>

double convertPlanarQuat2Psi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double psi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return psi;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "eta3_spline"); //node name
    ros::NodeHandle nh;
    ros::Publisher desired_state_publisher = nh.advertise<nav_msgs::Odometry>("/desState", 1);
    ros::Publisher x_publisher = nh.advertise<std_msgs::Float32>("/x_position", 1);
    ros::Publisher y_publisher = nh.advertise<std_msgs::Float32>("/y_position", 1);
    ros::Publisher theta_publisher = nh.advertise<std_msgs::Float32>("/des_theta_position", 1);
    std::vector<double> eta;
    std::vector<double> kappa;
    double d_t;
    bool got_traj;
    std::vector<geometry_msgs::PointStamped> poses;
    Eta3Traj Et;
    geometry_msgs::PointStamped pose;
    std::vector<nav_msgs::Odometry> traj;
    std::vector<nav_msgs::Odometry> traj_t;
    eta.resize(6);
    kappa.resize(4);
    int n;

    eta[0]=0.5;
    eta[1]=0.5;
    eta[2]=0;
    eta[3]=0;
    eta[4]=0;
    eta[5]=0;
    Et.setEta(eta);

    poses.clear();
    pose.header.frame_id = "des_frame";
    pose.point.x = 0;
    pose.point.y = 0;
    pose.point.z = 0;
    poses.push_back(pose);

    pose.point.x = 10;
    pose.point.y = 0;
    pose.point.z = 0.78;
    poses.push_back(pose);

    pose.point.x = 10;
    pose.point.y = 10;
    pose.point.z = 2.3;
    poses.push_back(pose);

    pose.point.x = 0;
    pose.point.y = 10;
    pose.point.z = -2.3;
    poses.push_back(pose);

    pose.point.x = 0;
    pose.point.y = 0;
    pose.point.z = 0;
    poses.push_back(pose);

    got_traj=false;
    Et.setPoses(poses);
    got_traj = Et.getTraj(traj,d_t); // It cannot be used to set the d_t but get the d_t
    if(!got_traj)
    {
    	ROS_ERROR("Failed to get the trajectory!");
    	return 0;
    }
    n = traj.size();
    for(int j=0; j<n; j++)
    {
    	traj_t.push_back(traj[j]);
    }


    /*nav_msgs::Path g_path;
	geometry_msgs::PoseStamped g_pose;
    ros::Publisher path_publisher = nh.advertise<nav_msgs::Path>("/path", 1);
    n = traj_t.size();
    nav_msgs::Odometry odom;
    g_path.header.frame_id = "odom_frame";
    for (int j=0; j<n; j++)
    {
    	odom = traj_t[j];
    	g_pose.pose = odom.pose.pose;
		g_path.poses.push_back(g_pose);
    }
    while(ros::ok())
    {
    	path_publisher.publish(g_path);
    	ros::Duration(0.02).sleep();
    }*/
    Et.runTraj2(traj);
    return 0;
}
