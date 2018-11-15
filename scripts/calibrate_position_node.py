#!/usr/bin/env python

import rospy
import dvrk
import numpy as np
import geometry_msgs.msg as gm
from force_observer.calibrate_position import CalibratePosition

if __name__ == '__main__':
    rosnode_name = 'calibrate_pose'
    rospy.init_node(rosnode_name, anonymous=True)

    accel_name = '/accel_calibrated'
    robot_joint_topic = '/dvrk/PSM1/state_joint_current'
    cal_pos = CalibratePosition(accel_name, robot_joint_topic)

    p = dvrk.psm('PSM1')
    #p.home()

    rospy.spin()


