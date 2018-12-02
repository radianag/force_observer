//
// Created by radian on 11/17/18.
//
#ifndef FORCE_OBSERVER_UTILS_H
#define FORCE_OBSERVER_UTILS_H

#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

Eigen::MatrixXd readCSV(std::string file, int rows, int cols);
Eigen::Matrix4d modified_dh(const double alpha, const double a, const double d, const double theta );


#endif //FORCE_OBSERVER_UTILS_H
