//
// Created by radian on 9/25/18.
//

#include "force_observer/observer.h"

Observer::Observer(std::shared_ptr<ros::NodeHandle> nhandle, bool is_calibrated): nh(nhandle){

    //if_calibrate = ~is_calibrated;
    if_calibrate = true;
    xyz_calibrate = 1;

    imu_sub = nh->subscribe("/accel", 10, &Observer::imu_cb, this);

}

void Observer::imu_cb(const std_msgs::Int16MultiArray &msg) {

    for (int i = 0; i<msg.layout.dim[0].size; ++i) imu_data[i] = msg.data[i];

}

void Observer::loop(){
    if (if_calibrate){

        this->calibrate();

    }

    else{
        int x = 0;
    }
}

void Observer::calibrate(){

    switch (xyz_calibrate)
    {
        case 1: {
            ROS_INFO_STREAM("Yaw and Pitch 0");
        }
        case 2: {
            ROS_INFO_STREAM("Yaw 90");
        } // code to be executed if n = 1;

        case 3: {
            ROS_INFO_STREAM("Pitch 90");
        }
        default:{
            if_calibrate = false;
        }
    }

}