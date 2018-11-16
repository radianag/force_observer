#!/usr/bin/env python

import rospy
import numpy as np
import geometry_msgs.msg as gm
import sensor_msgs.msg  as sm
import math as mt
import copy


class CalibrateImuPosition:
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
        self.data_joint_accel = []

        for i in range(3):
            self.data_accel.append(np.zeros([1, 3]))
            self.data_joint_pos.append(np.zeros([1, 2]))
            self.data_joint_accel.append(np.zeros([1, 2]))

        self.imu_distance = np.zeros(3)
        self.case_distance = np.zeros(3)

        self.check = [False, False, False]

    def callback_imu_calibrated(self, msg):
        self.imu_accel[0] = msg.linear.x
        self.imu_accel[1] = msg.linear.y
        self.imu_accel[2] = msg.linear.z

    def callback_joint_robot(self, msg):
        self.robot_joint_pos = msg.position[0:2]

        # This is making the Calibration code dependent
        # Hard coded acceleration calculation

        self.robot_joint_velocity_min1 = copy.deepcopy(self.robot_joint_velocity)
        self.robot_joint_velocity = msg.velocity[0:2]
        self.robot_joint_accel = np.subtract(self.robot_joint_velocity, self.robot_joint_velocity_min1)

    def _delete_gravity(self):
        x = 0

    def take_data(self, case_num):
        if self.data_accel[case_num].shape[0] == 1 and self.check[case_num]== False:
            self.data_accel[case_num][0] = self.imu_accel
            self.data_joint_accel[case_num][0] = self.robot_joint_accel
            self.check[case_num] = True
        else:
            self.data_accel[case_num] = np.concatenate((self.data_accel[case_num], [self.imu_accel]), axis=0)
            self.data_joint_accel[case_num] = np.concatenate((self.data_joint_accel[case_num], [self.robot_joint_accel]), axis=0)


    def solve_case(self, case_num, threshold=0.1):
        distance_array = []
        for i in range(self.data_accel[case_num].shape[0]):
            d = None
            accel = self.data_accel[case_num][i]
            joint_accel = self.data_joint_accel[case_num][i]

            if np.linalg.norm(accel) > threshold:
                if case_num == 0:
                    d = mt.sqrt(mt.pow(accel[1], 2) + mt.pow(accel[2], 2))/joint_accel[0]
                    #print(d)
                elif case_num == 1:
                    d = mt.sqrt(mt.pow(accel[1], 2) + mt.pow(accel[0], 2)) / joint_accel[1]
                else:
                    numerator = mt.sqrt(mt.pow(accel[0], 2) + mt.pow(accel[1], 2) + mt.pow(accel[2], 2))
                    denomenator = mt.sqrt(mt.pow(joint_accel[0], 2) + mt.pow(joint_accel[1], 2))
                    d = numerator/denomenator

                distance_array.append(d)


        self.case_distance[case_num] = np.mean(distance_array)
        print('case_dist ', case_num , self.case_distance[case_num])

    def solve_poses(self):
        case_d_sqr2 = np.power(self.case_distance, 2)
        A = np.matrix([[1, 0, 1], [0, 1, 1], [1, 1, 1]])

        imu_dist_sqr2 = np.matmul(np.linalg.inv(A), case_d_sqr2)
        self.imu_distance = np.sqrt(imu_dist_sqr2)

