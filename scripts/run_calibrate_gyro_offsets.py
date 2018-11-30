#!/usr/bin/env python

import rospy
import numpy as np

from force_observer.imu_data_processing import ImuDataProcessing
from force_observer.dynamixel_controller import DynamixelController
from force_observer.solver_ls_Ab import SolverlsAb
from force_observer.utils import *


if __name__ == '__main__':
    rospy.init_node('calibration', anonymous=True)

    #Settings
    ctrl_rate = 100  # [Hz]
    stabil_ts = 2 #[sec]
    delay_no_data = 4

    #Objects
    dynamixel = DynamixelController()
    imu = ImuDataProcessing(adc_topic='/accel')
    calib_gyro = SolverlsAb()

    r = rospy.Rate(ctrl_rate)
    rospy.sleep(1)

    # Tests repitition
    orientation_num = 3
    orientation = ['x', 'y', 'z']
    test_num = 4
    vel_array = np.linspace(1, 3, 3)
    diff_vel_num = len(vel_array)
    num_points = diff_vel_num * orientation_num * test_num

    for cur_orient in range(orientation_num):
        console_message = 'Put IMU in {0} orientation, Lay servo flat, then press any Key'.format(orientation[orientation_num])
        text = raw_input(console_message)
        print "Starting Test"
        for cur_vel in range(diff_vel_num):
            for cur_test in range(test_num):
                count = 0
                calib_gyro.cur_num_data = 0
                next_avg = True
                while next_avg and rospy.is_shutdown():
                    if calib_gyro.avg_cnt == 0:
                        if test_num%2 == 0:
                            dynamixel.publish_velocity(vel_array[cur_vel])
                        else:
                            dynamixel.publish_velocity(-vel_array[cur_vel])
                        rospy.sleep(delay_no_data)

                    vel = dynamixel.return_velocity()
                    known_vel = np.zeros(3)
                    known_vel[cur_orient] = vel

                    calib_gyro.set_data(known_vel, imu.return_gyro())
                    if count >= ctrl_rate*stabil_ts:
                        calib_gyro.get_avg_after_num()
                        next_avg = False
                    r.sleep()
                    count = count + 1

    calib_gyro.solve_LS()
    calib_gyro.make_csv('gyro_offsets')
    print('test_done')
