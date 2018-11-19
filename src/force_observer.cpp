//
// Created by radian on 11/17/18.
//

#include "force_observer.h"

ForceObserver::ForceObserver(const std::string imu_topic, const std::string robot_topic, const int num_dof) {

    // Matrix Placeholder Initializers
    dof = num_dof;

    robot_joint_pos.resize(dof);
    robot_joint_vel.resize(dof);
    robot_joint_eff.resize(dof);

    Ja_imu.resize(3, dof);
    Jd_imu.resize(3, dof);

    N.resize(dof); Fr.resize(dof);
    M.resize(dof, dof), Ja_robot.resize(3, dof), Jd_robot.resize(3, dof);

    // Ros Initializers
    std::shared_ptr<ros::NodeHandle> nhandle = std::make_shared<ros::NodeHandle>();

    // This is the robot interface part, Makes force observer dependent?
    imu_sub = nhandle->subscribe(imu_topic, 10, &ForceObserver::callback_imu_calibrated, this);
    robot_joint_sub = nhandle->subscribe(robot_topic, 10, &ForceObserver::callback_joint_robot, this);
    force_pub  = nhandle->advertise<geometry_msgs::Wrench>("/force_observer/tip_forces", 1);

    // Robot Dynamics
    rbt_dynamics = RobotDynamics();
}

void ForceObserver::callback_imu_calibrated(const geometry_msgs::Accel &msg){
    accel_data(0) = msg.linear.x;
    accel_data(1) = msg.linear.y;
    accel_data(2) = msg.linear.z;

    gyro_data(0) = msg.angular.x;
    gyro_data(1) = msg.angular.y;
    gyro_data(2) = msg.angular.z;
}

void ForceObserver::callback_joint_robot(const sensor_msgs::JointState &msg) {
    robot_joint_pos(0) = msg.position[0];
    robot_joint_pos(1) = msg.position[1];
    robot_joint_pos(2) = msg.position[2];

    robot_joint_vel(0) = msg.velocity[0];
    robot_joint_vel(1) = msg.velocity[1];
    robot_joint_vel(2) = msg.velocity[2];

    robot_joint_eff(0) = msg.effort[0];
    robot_joint_eff(1) = msg.effort[1];
    robot_joint_eff(2) = msg.effort[2];
}

void ForceObserver::setup_position(const std::string imu_pos_filename, int rows, int cols){
    Eigen::MatrixXd imu_pos;
    imu_pos.resize(rows, cols);
    imu_pos = readCSV(imu_pos_filename, rows, cols);

    imu_position(0) = imu_pos(1, 0);
    imu_position(1) = imu_pos(0, 1);
    imu_position(2) = 0;

    this->calculate_jacobian_imu();
}

void ForceObserver::calculate_jacobian_imu() {
    // Calculate Ja_imu from imu_position

    // Calculate Jd_imu from imu_position

}

void ForceObserver::calculate_dynamics() {

    M = rbt_dynamics.calcM(robot_joint_pos);
    N = rbt_dynamics.calcN(robot_joint_pos, robot_joint_vel);
    Fr = rbt_dynamics.calcFr(robot_joint_pos, robot_joint_vel);

    Ja_imu = rbt_dynamics.calcJa_imu(robot_joint_pos);
    Jd_imu = rbt_dynamics.calcJd_imu(robot_joint_pos, robot_joint_vel);

    Ja_robot = rbt_dynamics.calcJa(robot_joint_pos);
    Jd_robot = rbt_dynamics.calcJd(robot_joint_pos, robot_joint_vel);
}

Eigen::VectorXd ForceObserver::calculate_force() {
    Eigen::VectorXd Te, fe;
    Te.resize(3), fe.resize(3);

    Te = robot_joint_eff - M * Ja_imu.inverse() * (accel_data - Jd_imu*robot_joint_vel) -N - Fr;

    return Te;
}

Eigen::VectorXd ForceObserver::calculate_joint_accel() {
    Eigen::VectorXd Te;
    Te.resize(3);

    Te = Ja_imu.inverse()*(accel_data - Jd_imu*robot_joint_vel);

    return Te;
}

void ForceObserver::publish_accel(Eigen::VectorXd fe){
    force_msg.force.x = fe(0);
    force_msg.force.y = fe(1);
    force_msg.force.z = fe(2);

    force_pub.publish(force_msg);
}


void ForceObserver::publish_force(Eigen::VectorXd fe){
    force_msg.force.x = fe(0);
    force_msg.force.y = fe(1);
    force_msg.force.z = fe(2);

    force_pub.publish(force_msg);
}