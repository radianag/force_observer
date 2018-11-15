#!/usr/bin/env python

import rospy
import numpy as np
from force_observer.imu_data_processing import ImuDataProcessing

if __name__ == '__main__':
    name =  'test_offset'
    offset_name = 'imu_offsets'
    ctrl_rate = 100

    rospy.init_node(name, anonymous=True)
    r = rospy.Rate(ctrl_rate)

    offsets = np.genfromtxt(offset_name + '.csv', delimiter=',')

    imu = ImuDataProcessing()
    imu.setup_calibrated_publisher(offsets)

    while not rospy.is_shutdown():
        imu.publish_calibrated()
        r.sleep()

    rospy.spin()