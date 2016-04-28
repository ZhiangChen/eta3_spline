/// The monitor for library Eta3Traj
/// Zhiang Chen, 4/2016

#include <ros/ros.h> 
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
double convertPlanarQuat2Psi(geometry_msgs::Quaternion quaternion);

std_msgs::Float32 g_x;
std_msgs::Float32 g_y;
std_msgs::Float32 g_theta;
std_msgs::Float32 g_v;
std_msgs::Float32 g_o;
nav_msgs::Path g_path;
geometry_msgs::PoseStamped g_pose;
bool g_got_odom = false;

void monitorCallback(const nav_msgs::Odometry odom)
{
	g_got_odom = true;
	g_x.data = odom.pose.pose.position.x;
	g_y.data = odom.pose.pose.position.y;
	g_theta.data = convertPlanarQuat2Psi(odom.pose.pose.orientation);
	g_v.data = odom.twist.twist.linear.x;
	g_o.data = odom.twist.twist.angular.z;
	
	g_pose.pose = odom.pose.pose;
	g_path.poses.push_back(g_pose);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "eta3_monitor"); //node name
    ros::NodeHandle nh;

    ros::Subscriber eta3_subscriber= nh.subscribe("/odom",1,monitorCallback); 
    ros::Publisher x_publisher = nh.advertise<std_msgs::Float32>("/x_position", 1);
    ros::Publisher y_publisher = nh.advertise<std_msgs::Float32>("/y_position", 1);
    ros::Publisher theta_publisher = nh.advertise<std_msgs::Float32>("/theta_position", 1);
    ros::Publisher v_publisher = nh.advertise<std_msgs::Float32>("/velocity", 1);
    ros::Publisher o_publisher = nh.advertise<std_msgs::Float32>("/omega", 1);
    ros::Publisher path_publisher = nh.advertise<nav_msgs::Path>("/path", 1);

    g_path.header.frame_id = "odom_frame";
    while(!g_got_odom)  
    {
    	ROS_INFO("waiting for odom");
    	ros::spinOnce();
    	ros::Duration(0.5).sleep();
    }
    ROS_INFO("Starting publishing...");
    ROS_INFO("The topic names are: /x_position, /y_position, /theta_position, /velocity, /omega");
    while(ros::ok())
    {
    	ros::spinOnce();
    	x_publisher.publish(g_x);
    	y_publisher.publish(g_y);
    	theta_publisher.publish(g_theta);
    	v_publisher.publish(g_v);
    	o_publisher.publish(g_o);
    	path_publisher.publish(g_path);
    	ros::Duration(0.02).sleep();
    }

}

double convertPlanarQuat2Psi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double psi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return psi;
}
