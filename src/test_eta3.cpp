/// The test main file for library Eta3Traj
/// Zhiang Chen, 4/2016

#include<ros/ros.h> 
#include <eta3_spline/eta3_spline.h>
#include <geometry_msgs/PointStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "eta3_spline"); //node name
    Eta3Traj Et;
    std::vector<geometry_msgs::PointStamped> poses;
    poses.clear();
    geometry_msgs::PointStamped pose;
    pose.header.frame_id = "des_frame";
    pose.point.x = 0;
    pose.point.y = 0;
    pose.point.z = 0;
    poses.push_back(pose);

    pose.point.x = 5;
    pose.point.y = 2;
    pose.point.z = 1;

    while (ros::ok()) {
    }

    return 0;
}
