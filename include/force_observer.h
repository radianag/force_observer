//
// Created by radian on 11/17/18.
//

#ifndef FORCE_OBSERVER_FORCE_OBSERVER_H
#define FORCE_OBSERVER_FORCE_OBSERVER_H

#include <ros/ros.h>
#include <memory>
#include <string.h>

#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "robot_dynamics.h"
#include "utils.h"

class ForceObserver {
private:
    ros::Subscriber imu_sub, robot_joint_sub;
    ros::Publisher  force_pub;

    geometry_msgs::Wrench force_msg;

    RobotDynamics rbt_dynamics;

    int dof;
    Eigen::Vector3d accel_data, gyro_data;
    Eigen::Vector3d imu_position;
    Eigen::MatrixXd Ja_imu, Jd_imu;

    Eigen::VectorXd robot_joint_pos, robot_joint_vel, robot_joint_eff;

    Eigen::VectorXd N, Fr;
    Eigen::MatrixXd M, Ja_robot, Jd_robot;

    void callback_imu_calibrated(const geometry_msgs::Accel &msg);
    void callback_joint_robot(const sensor_msgs::JointState &msg);
    void calculate_jacobian_imu();

public:
    ForceObserver(const std::string imu_topic, const std::string robot_topic, const int num_dof);
    void setup_position(const std::string imu_pos_filename, int rows, int cols);
    void calculate_dynamics();
    Eigen::VectorXd calculate_force();
    void publish_force(Eigen::VectorXd fe);

};

#endif //FORCE_OBSERVER_FORCE_OBSERVER_H
