#!/usr/bin/env python

import rospy
import dvrk
import numpy as np
import geometry_msgs.msg as gm
import matplotlib.pyplot as plt
from force_observer.calibrate_position import CalibratePosition
from force_observer.interpolation import Interpolation

if __name__ == '__main__':
    rosnode_name = 'calibrate_pose'
    rospy.init_node(rosnode_name, anonymous=True)

    accel_name = '/accel_calibrated'
    robot_joint_topic = '/dvrk/PSM1/state_joint_current'
    test = CalibratePosition(accel_name, robot_joint_topic)

    p = dvrk.psm('PSM1')
    p.home()

    # Settings
    ctrl_rate = 100  # [Hz]



    limit = [(-1.54, 1.54), (-0.8, 0.8)]
    q3_pos = 0.05

    r = rospy.Rate(ctrl_rate)
    rospy.sleep(1)

    case_num = 3
    trial_num = 2

    interp = Interpolation()

    # Trajectory
    num_pts = 350
    v0 = [0.0]
    vf = [0.0]
    a0 = [0.0]
    af = [0.0]
    t0 = 0.0
    tf = 5

    case_i = 0
    count = 0
    for z in range(2):
        for j in range(trial_num):
            if count % 2 == 0:
                p0 = [limit[case_i][0]]
                pf = [limit[case_i][1]]
            else:
                p0 = [limit[case_i][1]]
                pf = [limit[case_i][0]]

            interp.compute_interpolation_params(p0, pf, v0, vf, a0, af, t0, tf)
            t = np.linspace(0, tf, num_pts)
            pos = interp.get_interpolated_x(t)

            if case_i == 0:
                p.move_joint_some(np.array([pos[0, 0], 0, 0]), np.array([0, 1, 2]))
            else:
                p.move_joint_some(np.array([0, pos[0, 0], 0]), np.array([0, 1, 2]))

            for i in range(num_pts):
                if case_i == 0:
                    p.move_joint_some(np.array([pos[0, i], 0]), np.array([0, 1]), interpolate=False, blocking=False)
                else:
                    p.move_joint_some(np.array([0, pos[0, i]]), np.array([0, 1]), interpolate=False, blocking=False)
                test.take_data(case_i)
                r.sleep()
            count = count + 1
        case_i = case_i + 1

    case_i = 2
    count = 0

    limit = [(-0.8, 0.8), (-0.8, 0.8)]
    v0 = [ 0.0, 0.0]
    vf = [ 0.0, 0.0]
    a0 = [ 0.0, 0.0]
    af = [ 0.0, 0.0]

    for j in range(trial_num):
        if count % 2 == 0:
            p0 = [limit[0][0], limit[1][0]]
            pf = [limit[0][1], limit[1][1]]
        else:
            p0 = [limit[0][1], limit[1][1]]
            pf = [limit[0][0], limit[1][0]]

        interp.compute_interpolation_params(p0, pf, v0, vf, a0, af, t0, tf)
        t = np.linspace(0, tf, num_pts)
        pos = interp.get_interpolated_x(t)

        p.move_joint_some(np.array([pos[0, 0], pos[1, 0], 0]), np.array([0, 1, 2]))

        for i in range(num_pts):
            p.move_joint_some(np.array([pos[0, i], pos[1, i]]), np.array([0, 1]), interpolate=False, blocking=False)
            test.take_data(case_i)
            r.sleep()

        count = count + 1


    test.solve_case(0)
    print('case_dist',0,test.case_distance)
    test.solve_case(1)
    test.solve_case(2)
    test.solve_poses()
    print(test.imu_distance)


