//
// Created by radian on 11/17/18.
//
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <string.h>

#include <ros/ros.h>

#include "force_observer.h"

int main(int argc, char **argv)
{

    std::string imu_topic = "/accel_calibrated";
    std::string robot_topic = "/dvrk/PSM1/state_joint_current";
    std::string imu_pos_filename = "/data/imu_position.csv";
    int dof = 3;

    ros::init(argc, argv, "force_observer_node");
    ros::Rate node_rate(100);

    ForceObserver observer_node = ForceObserver(imu_topic, robot_topic, dof);
    observer_node.setup_position(imu_pos_filename, 2, 3);

    while(ros::ok())
    {

        //observer_node.publish_force();
        ros::spinOnce();
        node_rate.sleep();
    }
    return 0;

}