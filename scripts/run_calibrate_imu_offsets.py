#!/usr/bin/env python

import rospy
import numpy as np
import dvrk
from force_observer.calibrate_imu_offset import CalibrateImuOffset
from force_observer.utils import *


if __name__ == '__main__':
    rospy.init_node('calibration', anonymous=True)

    #Settings
    ctrl_rate = 100  # [Hz]
    stabil_ts = 2 #[sec]

    # Calibrate to the principal axis of the IMU,
    # The combined Axis is not good

    q1_num_pts = 3
    q2_num_pts = 3

    limit = [(-1.54, 1.54), (-0.8, 0.8)]
    q3_pos = 0.05

    r = rospy.Rate(ctrl_rate)

    traj_q1 = np.linspace(limit[0][0], limit[0][1], num=q1_num_pts)
    traj_q2 = np.linspace(limit[1][0], limit[1][1], num=q2_num_pts)

    num_points = q1_num_pts*q2_num_pts

    p = dvrk.psm('PSM1')
    p.home()
    rospy.sleep(1)

    imu = CalibrateImuOffset()
    imu.setup_calibrate(ctrl_rate*stabil_ts)

    # Trajectory at different position of robot
    state1 = np.linspace(0, q1_num_pts-1, q1_num_pts, dtype=int)
    state2 = np.linspace(0, q2_num_pts-1, q2_num_pts, dtype=int)

    flat_state = [(0, 0)] * num_points
    for i in range(q1_num_pts):
        for j in range(q2_num_pts):
            flat_state[i * q2_num_pts+j] = (state1[i], state2[j])
    print(traj_q1)

    a = [0, 0]

    # Go to trajectory, hold, get rotation, set data for calibration
    while (imu.state < num_points) and not rospy.is_shutdown():

        if imu.avg_cnt == 0:
            p.move_joint_some(np.array([traj_q1[flat_state[imu.state][0]], traj_q2[flat_state[imu.state][1]], q3_pos]), np.array([0, 1, 2]))
            rospy.sleep(4)
        a = p.get_current_joint_position()[0:2]
        yaw = a[0]
        pitch = a[1]

        # FRAME 3 Main Insertion, This should be modular and encapsulated and saved in a main data of the user
        R1 = modified_dh(np.pi/2, 0, 0, yaw + np.pi/2) * modified_dh(-np.pi/2, 0, 0, pitch - np.pi/2) * modified_dh(np.pi/2, 0, 0, 0)
        #R1 = tf.transformations.euler_matrix(yaw, pitch, 0, 'syxz')
        R2 = R1[0:3][0:3].transpose()
        imu.set_data(R2)
        imu.get_avg()
        r.sleep()

    imu.solve_LS()
    print('test_done')
