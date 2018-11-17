#!/usr/bin/env python

import rospy
import numpy as np
import std_msgs.msg
import geometry_msgs.msg as gm
import sensor_msgs.msg  as sm
from force_observer.utils import *

class ImuDataProcessing:
    def __init__(self):
        self.name = "IMU"
        self.sub = rospy.Subscriber('/accel', std_msgs.msg.Int16MultiArray, self.callback_imu)
        self.pub = rospy.Publisher('/accel_calibrated', gm.Accel, queue_size=1)
        self.sub_dvrk = rospy.Subscriber('/dvrk/PSM1/state_joint_current', sm.JointState, self.callback_joint_robot)

        self.data = np.zeros(7)
        self.g = np.array([0, 0, -9.81])

        self.offsets = []
        self.new_msg = gm.Accel()
        self.robot_joint_pos = np.zeros(2)

    def callback_imu(self, msg):
        scale = 2.0 * 9.81 / 32767.0
        self.data = np.array(msg.data, dtype=float)
        self.data[1:] = self.data[1:] * scale
        #print(self.data)

    def callback_joint_robot(self, msg):
        self.robot_joint_pos = msg.position[0:2]

    def delete_gravity(self, x):
        # Dependent on the DH frame of the IMU, should be modular
        R1 = modified_dh(np.pi/2, 0, 0, self.robot_joint_pos[0] + np.pi/2) * modified_dh(-np.pi/2, 0, 0, self.robot_joint_pos[1] - np.pi/2)

        grav = np.matmul(R1.transpose(), np.append(self.g, 1))
        x = np.subtract(x, grav[0, 0:3])
        return x

    def publish_calibrated(self):
        y = self.data[1:4] - self.offsets[-3:]

        A = np.matrix([self.offsets[0:3], self.offsets[3:6], self.offsets[6:9]])
        A = np.transpose(A)
        x = np.matmul(np.linalg.inv(A), y)

        # Comment to have gravity effect
        x = self.delete_gravity(x[0, 0:3])

        self.new_msg.linear.x = x[0, 0]
        self.new_msg.linear.y = x[0, 1]
        self.new_msg.linear.z = x[0, 2]

        self.pub.publish(self.new_msg)

    def setup_calibrated_publisher(self, offsets):
        self.offsets = offsets