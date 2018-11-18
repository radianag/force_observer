//
// Created by radian on 11/17/18.
//
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>

int main(int argc, char **argv)
{
    Eigen::VectorXd x;

    x.resize(3);
    x << 1, 2, 3;
    std::cout<< x.size();
}