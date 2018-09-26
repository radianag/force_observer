//
// Created by radian on 9/25/18.
//

#ifndef PROJECT_OBSERVER_H
#define PROJECT_OBSERVER_H

#include <iostream>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory>



#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>

using namespace std;

class Observer {
private:

    ros::Subscriber imu_sub;
    ros::Publisher force_pub;

    int16_t imu_data[7];

    bool if_calibrate;
    int xyz_calibrate ;

    shared_ptr<ros::NodeHandle> nh;

public:
    Observer(std::shared_ptr<ros::NodeHandle> nhandle, bool is_calibrated);
    void imu_cb(const std_msgs::Int16MultiArray &msg);
    void calibrate();
    void loop();
};

#endif //PROJECT_OBSERVER_H