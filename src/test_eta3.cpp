/// The test main file for library Eta3Traj
/// Zhiang Chen, 4/2016

#include<ros/ros.h> 
#include <eta3_spline/eta3_spline.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "eta3_spline"); //node name
    Eta3Traj Et;

    while (ros::ok()) {
    }

    return 0;
}
