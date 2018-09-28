#!/usr/bin/env python

import rospy
import numpy as np
import tf

import dvrk
from force_observer.imu import Imu

def modified_dh(_dh_alpha, _dh_a, _dh_d, _dh_theta):
    A = np.matrix([[m.cos(_dh_theta), -m.sin(_dh_theta), 0, _dh_a],
    [m.sin(_dh_theta) * m.cos(_dh_alpha), m.cos(_dh_theta) * m.cos(_dh_alpha), -m.sin(_dh_alpha), -m.sin(_dh_alpha) * _dh_d],
    [m.sin(_dh_theta) * m.sin(_dh_alpha), m.cos(_dh_theta) * m.sin(_dh_alpha), m.cos(_dh_alpha), m.cos(_dh_alpha) * _dh_d],
    [0, 0, 0, 1]])

    return A 

if __name__ == '__main__':
    rospy.init_node('calibration', anonymous=True)

    #Settings
    ctrl_rate = 100       # [Hz]
    stabil_ts = 2 #[sec]
    
    q1_num_pts = 5
    q2_num_pts = 5

    limit = [(-1.54, 1.54), (-0.8, 0.8)]
    q3_pos = 0.05

    r = rospy.Rate(ctrl_rate)

    traj_q1 = np.linspace(limit[0][0], limit[0][1], num=q1_num_pts)
    traj_q2 = np.linspace(limit[1][0], limit[1][1], num=q2_num_pts)

    num_points = q1_num_pts*q2_num_pts

    p = dvrk.psm('PSM1')
    p.home()
    rospy.sleep(1)

    imu = Imu()
    imu.setup_calibrate(ctrl_rate*stabil_ts)

    #yaw = 1
    #pitch = 1.54

    state1 = np.linspace(0, q1_num_pts-1, q1_num_pts, dtype=int)
    state2 = np.linspace(0, q2_num_pts-1, q2_num_pts, dtype=int)

    flat_state = [(0, 0)] * num_points
    for i in range(q1_num_pts):
        for j in range(q2_num_pts):
            flat_state[i * q2_num_pts+j] = (state1[i], state2[j])
    print(traj_q1)

    a = [0, 0]
    while (imu.state < num_points) and not rospy.is_shutdown():

        if imu.avg_cnt == 0:
            p.move_joint_some(np.array([traj_q1[flat_state[imu.state][0]], traj_q2[flat_state[imu.state][1]], q3_pos]), np.array([0, 1, 2]))
            rospy.sleep(6)
        a = p.get_current_joint_position()[0:2]
        yaw = a[0]
        pitch = a[1]
        R1 = modified_dh(np.pi/2, 0, 0, yaw + np.pi/2) * modified_dh(-np.pi/2, 0, 0, pitch - np.pi/2)
        #R1 = tf.transformations.euler_matrix(yaw, pitch, 0, 'syxz')
        imu.set_data(R1)
        imu.get_avg()
        r.sleep()

    imu.solve_LS()

    rospy.spin()