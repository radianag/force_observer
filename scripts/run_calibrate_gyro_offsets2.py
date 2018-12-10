#!/usr/bin/env python

import rospy
import numpy as np

from force_observer.imu_data_processing import ImuDataProcessing
from force_observer.dynamixel_controller import DynamixelController
from force_observer.solver_ls_Ab import SolverLsAb
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
    calib_gyro = SolverLsAb()

    r = rospy.Rate(ctrl_rate)
    rospy.sleep(1)

    # Tests repitition
    orientation_num = 3
    orientation = ['x', 'y', 'z']
    test_num = 1
    vel_array = np.linspace(1, 3, 3)
    diff_vel_num = len(vel_array)
    num_points = diff_vel_num * orientation_num * test_num

    keep_testing = True

    while keep_testing and not rospy.is_shutdown():
        console_message = 'Enter which axis to calibrate? [x, y, or z], press [n] to finish testing. After input, ...' \
                          ' test will start immediately.'
        text = raw_input(console_message)
        if text == 'n':
            break

        cur_orient = orientation.index(text)

        #console_message = 'Put IMU in {0} orientation, Lay servo flat, then press any Key'.format(orientation[orientation_num])
        #text = raw_input(console_message)

        #print "Starting Test, Press enter to toggle get/not get data"
        getting_orient = True
        getting_data = True
        count = 0
        data_point = 0
        while getting_orient and not rospy.is_shutdown():
            if getting_data == True:
                vel = dynamixel.return_velocity()
                known_vel = np.zeros(3)
                known_vel[cur_orient] = vel
                calib_gyro.set_data(known_vel, imu.return_gyro())

                if count >= ctrl_rate*stabil_ts:
                    calib_gyro.get_avg_after_num()
                    data_point = data_point + 1
                    #getting_orient = False

                if data_point >= test_num:
                    getting_orient = False
                    print "This Orientation data done"

                r.sleep()
                count = count + 1

    calib_gyro.solve_LS()
    calib_gyro.make_csv('gyro_offsets')
    print('test_done')
