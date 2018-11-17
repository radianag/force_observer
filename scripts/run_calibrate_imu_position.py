#!/usr/bin/env python

import rospy
import dvrk
import numpy as np
import geometry_msgs.msg as gm
import matplotlib.pyplot as plt
from force_observer.calibrate_imu_position_velocity import CalibrateImuPositionVelocity
from force_observer.interpolation import Interpolation
import math as mt
from force_observer.utils import *

if __name__ == '__main__':
    rosnode_name = 'calibrate_pose'
    rospy.init_node(rosnode_name, anonymous=True)

    accel_name = '/accel_calibrated'
    robot_joint_topic = '/dvrk/PSM1/state_joint_current'
    test = CalibrateImuPositionVelocity(accel_name, robot_joint_topic)

    p = dvrk.psm('PSM1')
    p.home()

    # Settings
    ctrl_rate = 150  # [Hz]

    limit = [(-1.5, -0.8), (-0.7, -0.3)]
    q3_pos = 0.05

    r = rospy.Rate(ctrl_rate)
    rospy.sleep(1)

    case_num = 1
    trial_num = 1

    interp = Interpolation()

    # Trajectory
    v0 = [0.0]

    vf = [0.0]
    a0 = [0.0]
    af = [0.0]
    t0 = 0.0

    vi = [1.4, 1.6]
    tf_array = [1, 0.5]
    num_pts = np.multiply(tf_array, ctrl_rate)

    hard_threshold = 10
    case_i = 0
    count = 0
    for z in range(2):
        for j in range(trial_num):
            vii = []
            pos_array = []
            interp = Interpolation()
            if count % 2 == 0:
                p0 = [limit[case_i][0]]
                pf = [limit[case_i][1]]
                vii = [vi[case_i]]
            else:
                p0 = [limit[case_i][0]]
                pf = [limit[case_i][1]]
                vii =[vi[case_i]]
            #print(vii)
            interp.compute_interpolation_params(p0, pf, v0, vii, a0, af, t0, tf_array[case_i])

            t = np.linspace(0, tf_array[case_i], num_pts[case_i])
            pos = interp.get_interpolated_x(t)
            pos_array = np.array(pos[0])

            lin_num_pts = mt.fabs(pf[0] * 2)/mt.fabs(vii[0]) * ctrl_rate
            pos = np.linspace(pf[0], -pf[0], lin_num_pts)

            pos_array = np.append(pos_array, pos[1:-1])

            interp.compute_interpolation_params([-pf[0]], [-p0[0]], vii, vf, a0, af, t0, tf_array[case_i])
            pos = interp.get_interpolated_x(t)

            pos_array = np.append(pos_array, pos[0])
            #print('pos_array ', j , ' asdf ', pos_array)

            xf = np.linspace(0,1, pos_array.shape[0])
            #plt.plot(xf, pos_array, '.')

            if case_i == 0:
                p.move_joint_some(np.array([pos_array[0], 0, 0]), np.array([0, 1, 2]))
            else:
                p.move_joint_some(np.array([0, pos_array[0], 0]), np.array([0, 1, 2]))

            for i in range(pos_array.shape[0]):
                #print(pos_array[i])
                if case_i == 0:
                    p.move_joint_some(np.array([pos_array[i], 0]), np.array([0, 1]), interpolate=False, blocking=False)
                else:
                    p.move_joint_some(np.array([0, pos_array[i]]), np.array([0, 1]), interpolate=False, blocking=False)

                if i > num_pts[case_i]+hard_threshold and i <num_pts[case_i] + lin_num_pts-hard_threshold:
                    test.take_data(case_i)
                r.sleep()
            count = count + 1
        case_i = case_i + 1
    plt.show()
    threshold = 0.8
    test.solve_case(0, threshold)
    test.solve_case(1, threshold)

    print(test.case_distance)
    save_csv_data('data/','imu_position', test.case_distance)
