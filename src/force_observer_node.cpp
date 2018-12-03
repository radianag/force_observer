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
    // Settings
    int dof = 3;
    std::string imu_topic = "/accel_calibrated";
    std::string robot_topic = "/dvrk/PSM1/state_joint_current";
    std::string imu_pos_filename = "/data/imu_position.csv";
    int sample_rate = 100;

    // Ros Initializers
    ros::init(argc, argv, "force_observer_node");


    // Observer Initializers
    ForceObserver observer_node = ForceObserver(imu_topic, robot_topic, dof);
    observer_node.setup_position(imu_pos_filename, 2, 3);

    ros::Rate node_rate(sample_rate);
    Eigen::Vector3d joint_accel, joint_torque, fe;

    while(ros::ok())
    {
        observer_node.calculate_dynamics();
        joint_accel = observer_node.calculate_joint_accel();
        joint_torque = observer_node.calculate_torque_from_imu();
        fe = observer_node.calculate_force_estimate(joint_torque);

        observer_node.publish_force(fe);
        observer_node.publish_accel(joint_accel);
        ros::spinOnce();
        node_rate.sleep();
    }
    return 0;

}
