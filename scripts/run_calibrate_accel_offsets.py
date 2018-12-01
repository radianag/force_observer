#!/usr/bin/env python

import rospy
import numpy as np
import dvrk
from force_observer.solver_ls_Ab import SolverlsAb
from force_observer.imu_data_processing import ImuDataProcessing
from force_observer.utils import *


if __name__ == '__main__':
    rospy.init_node('calibration', anonymous=True)

    #Settings
    ctrl_rate = 100  # [Hz]
    stabil_ts = 2 #[sec]
    delay_no_data = 4

    gravity = np.array([0, 0, -9.81])

    # Calibrate to the principal axis of the IMU,
    # The combined Axis is not good
    q1_num_pts = 3
    q2_num_pts = 3

    limit = [(-1.54, 1.54), (-0.8, 0.8)]
    q3_pos = 0.05

    r = rospy.Rate(ctrl_rate)

    traj_q1 = np.linspace(limit[0][0], limit[0][1], num=q1_num_pts)
    traj_q2 = np.linspace(limit[1][0], limit[1][1], num=q2_num_pts)
    #traj_q2 = np.zeros(1)

    num_points = q1_num_pts*q2_num_pts

    # Hardare Interfaces
    p = dvrk.psm('PSM1')
    p.home()
    imu = ImuDataProcessing(adc_topic='/accel')

    # LS Solver
    calib_accel = SolverlsAb()
    rospy.sleep(1)

    # Trajectory at different position of robot
    state1 = np.linspace(0, q1_num_pts-1, q1_num_pts, dtype=int)
    state2 = np.linspace(0, q2_num_pts-1, q2_num_pts, dtype=int)
    #state2 = np.array([0])

    flat_state = [(0, 0)] * num_points
    for i in range(q1_num_pts):
        for j in range(q2_num_pts):
            flat_state[i * q2_num_pts+j] = (state1[i], state2[j])
    print(flat_state)

    a = [0, 0]

    # Go to trajectory, hold, get rotation, set data for calibration
    for i in range(2):
        calib_accel.cur_num_data = 0
        next_avg = True
        count = 0
        while next_avg and not rospy.is_shutdown():
            if calib_accel.avg_cnt == 0:
                p.move_joint_some(np.array([traj_q1[flat_state[calib_accel.cur_num_data][0]], traj_q2[flat_state[calib_accel.cur_num_data][1]], q3_pos]), np.array([0, 1, 2]))
                rospy.sleep(delay_no_data)
            a = p.get_current_joint_position()[0:2]
            yaw = a[0]
            pitch = a[1]

            # FRAME 3 Main Insertion, This should be modular and encapsulated and saved in a main data of the user
            R1 = modified_dh(np.pi/2, 0, 0, yaw + np.pi/2) * modified_dh(-np.pi/2, 0, 0, pitch - np.pi/2) * modified_dh(np.pi/2, 0, 0, 0)
            #R1 = tf.transformations.euler_matrix(yaw, pitch, 0, 'syxz')
            R2 = R1[0:3][0:3].transpose()
            x = np.matmul(R2, gravity)
            calib_accel.set_data(x, imu.return_accel())

            if count >= ctrl_rate*stabil_ts:
                calib_accel.get_avg_after_num()
                next_avg = False

            count = count +1
            r.sleep()

    calib_accel.solve_LS()
    calib_accel.make_csv('accel_offsets')
    print('test_done')
