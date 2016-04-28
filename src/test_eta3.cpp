/// The test main file for library Eta3Traj
/// Zhiang Chen, 4/2016

#include<ros/ros.h> 
#include <eta3_spline/eta3_spline.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

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
    eta.resize(6);
    eta[0] = 5;
    eta[1] = 5;
    eta[2] = 5;
    eta[3] = 5;
    eta[4] = 5;
    eta[5] = 5;

    Eta3Traj Et;
    Et.setEta(eta);
    std::vector<geometry_msgs::PointStamped> poses;
    poses.clear();
    geometry_msgs::PointStamped pose;
    pose.header.frame_id = "des_frame";
    pose.point.x = 0;
    pose.point.y = 0;
    pose.point.z = 0;
    poses.push_back(pose);

    pose.point.x = 10;
    pose.point.y = 0;
    pose.point.z = 0.785;
    poses.push_back(pose);

    pose.point.x = 10;
    pose.point.y = 10;
    pose.point.z = 2.355;
    poses.push_back(pose);

    pose.point.x = 0;
    pose.point.y = 10;
    pose.point.z = -2.355;
    poses.push_back(pose);

    pose.point.x = 0;
    pose.point.y = 0;
    pose.point.z = 0;
    poses.push_back(pose);

    std::vector<nav_msgs::Odometry> traj;
    double d_t;
    bool got_traj=false;
    Et.setPoses(poses);
    got_traj = Et.getTraj(traj,d_t); // It cannot be used to set the d_t but get the d_t
    if(!got_traj)
    {
    	ROS_ERROR("Failed to get the trajectory!");
    	return 0;
    }
    Et.runTraj2();
    return 0;
}
