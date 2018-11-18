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
    Eigen::VectorXd CalcFr(const Eigen::VectorXd &q, const Eigen::VectorXd &qd);
    Eigen::VectorXd CalcN(const Eigen::VectorXd &q,const Eigen::VectorXd &qd);
    Eigen::MatrixXd CalcM(const Eigen::VectorXd &q);
    Eigen::MatrixXd CalcJd(const Eigen::VectorXd &q, const Eigen::VectorXd &qd);
    Eigen::MatrixXd CalcJa(const Eigen::VectorXd &q, const Eigen::VectorXd &qd);
};

#endif //FORCE_OBSERVER_DYNAMICS_H
