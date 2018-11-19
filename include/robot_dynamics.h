//
// Created by radian on 11/17/18.
//

#ifndef FORCE_OBSERVER_DYNAMICS_H
#define FORCE_OBSERVER_DYNAMICS_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>

class RobotDynamics{
public:
    RobotDynamics();
    Eigen::VectorXd calcFr(const Eigen::VectorXd &q, const Eigen::VectorXd &qd);
    Eigen::VectorXd calcN(const Eigen::VectorXd &q,const Eigen::VectorXd &qd);
    Eigen::MatrixXd calcM(const Eigen::VectorXd &q);
    Eigen::MatrixXd calcJd(const Eigen::VectorXd &q, const Eigen::VectorXd &qd);
    Eigen::MatrixXd calcJd_imu(const Eigen::VectorXd &q, const Eigen::VectorXd &qd);
    Eigen::MatrixXd calcJa(const Eigen::VectorXd &q);
    Eigen::MatrixXd calcJa_imu(const Eigen::VectorXd &q);
};

#endif //FORCE_OBSERVER_DYNAMICS_H
