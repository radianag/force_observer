#!/usr/bin/env python

import rospy
import numpy as np
import geometry_msgs.msg as gm
import sensor_msgs.msg  as sm
import math as mt
import copy


class CalibrateImuPositionVelocity:
    # @Description: Calibrates Imu Position [dx, dy, dz] from robot tip using
    # robot acceleration and imu acceleration
    def __init__(self, accel_name, robot_joint_topic):

        self.accel_name = accel_name
        self.robot_joint_topic = robot_joint_topic

        self.sub_imu = rospy.Subscriber(self.accel_name, gm.Accel, self.callback_imu_calibrated)
        self.sub_dvrk = rospy.Subscriber(self.robot_joint_topic, sm.JointState, self.callback_joint_robot)

        self.imu_accel = np.zeros(3)
        self.robot_joint_pos = np.zeros(2)
        self.robot_joint_velocity = np.zeros(2)
        self.robot_joint_velocity_min1 = np.zeros(2)
        self.robot_joint_accel = []

        self.data_accel = []
        self.data_joint_pos = []
        self.data_joint_velocity = []

        for i in range(3):
            self.data_accel.append(np.zeros([1, 3]))
            self.data_joint_pos.append(np.zeros([1, 2]))
            self.data_joint_velocity.append(np.zeros([1, 2]))

        self.imu_distance = np.zeros(3)
        self.case_distance = np.zeros((2, 3))

        self.check = [False, False, False]

    def callback_imu_calibrated(self, msg):
        self.imu_accel[0] = msg.linear.x
        self.imu_accel[1] = msg.linear.y
        self.imu_accel[2] = msg.linear.z

    def callback_joint_robot(self, msg):
        self.robot_joint_pos = msg.position[0:2]
        self.robot_joint_velocity = msg.velocity[0:2]

    def _delete_gravity(self):
        x = 0

    def take_data(self, case_num):
        if self.data_accel[case_num].shape[0] == 1 and self.check[case_num]== False:
            self.data_accel[case_num][0] = self.imu_accel
            self.data_joint_velocity[case_num][0] = self.robot_joint_velocity

            self.check[case_num] = True
        else:
            self.data_accel[case_num] = np.concatenate((self.data_accel[case_num], [self.imu_accel]), axis=0)
            self.data_joint_velocity[case_num] = np.concatenate((self.data_joint_velocity[case_num], [self.robot_joint_velocity]), axis=0)

    def solve_case(self, case_num, threshold=0.1):
        # Data already taken at const velocity

        accel = np.mean(self.data_accel[case_num], axis=0)
        joint_vel = np.mean(self.data_joint_velocity[case_num], axis=0)

        mag_accel = np.linalg.norm(accel)
        mag_joint_vel = joint_vel[case_num]
        mag_distance = mag_accel / mt.pow(mag_joint_vel, 2)

        if case_num == 0:
            self.case_distance[case_num][2] = accel[2]/mag_accel * mag_distance
            self.case_distance[case_num][1] = accel[1]/mag_accel * mag_distance
        else:
            self.case_distance[case_num][0] = accel[0]/mag_accel * mag_distance
            self.case_distance[case_num][1] = accel[1]/mag_accel * mag_distance

        print('imu distances: ', case_num , self.case_distance[case_num])


