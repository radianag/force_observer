#!/usr/bin/env python

import rospy
import numpy as np
import std_msgs.msg
import csv
import rospkg

class CalibrateImuOffset:
    def __init__(self):
        self.name = "IMU"
        self.sub = rospy.Subscriber('/accel', std_msgs.msg.Int16MultiArray, self.callback_imu)

        self.avg_data_pts = 0

        # Least Squares stuff
        self.Xb = np.zeros([3, 12])
        self.x_array = np.zeros([1, 3])

        self.yb = np.zeros([1, 3])
        self.y_array = np.zeros([1, 3])

        self.data = np.zeros(7)
        self.g = np.array([0, 0, -9.81])

        self.avg_cnt = 0
        self.state = 0

    def setup_calibrate(self, avg_data_points):
        self.avg_data_pts = avg_data_points

    def callback_imu(self, msg):
        #Hardcoded Gravity Scale
        scale = 2.0 * 9.81 / 32767.0

        self.data = np.array(msg.data, dtype=float)
        self.data[1:] = self.data[1:] * scale

    def set_data(self, R1):
        x = np.matmul(R1[0:3, 0:3], self.g)
        if self.avg_cnt == 0:
            self.x_array[0] = x
            self.y_array[0] = np.array([self.data[1], self.data[2], self.data[3]])

        else:
            self.x_array = np.concatenate((self.x_array, x), axis=0)
            self.y_array = np.concatenate((self.y_array, [[self.data[1], self.data[2], self.data[3]]]), axis=0)

        self.avg_cnt += 1

    def _make_X(self, x):
        R = [None] * 3
        for i in range(3):
            R[i] = np.eye(3) * x[i]
            if i == 0:
                X = R[i]
            else:
                X = np.concatenate((X, R[i]), axis=1)
        X = np.concatenate((X, np.eye(3)), axis=1)
        return X

    def _concatenate_Xb(self, X, y):
        if np.sum(self.Xb) == 0:
            self.Xb = X
            self.Yb = y
        else:
            self.Xb = np.concatenate((self.Xb, X), axis=0)
            self.Yb = np.concatenate((self.Yb, y))
        print("shapes", np.shape(self.Xb), np.shape(self.Yb))

    def get_avg(self):
        if self.avg_cnt > self.avg_data_pts:
            avg_x = np.sum(self.x_array, axis=0) / self.avg_cnt
            avg_y = np.sum(self.y_array, axis=0) / self.avg_cnt

            self.state += 1
            self.avg_cnt = 0
            self.x_array = np.zeros([1, 3])
            self.y_array = np.zeros([1, 3])

            X = self._make_X(np.array([avg_x[0, 0],avg_x[0, 1],avg_x[0, 2]]))
            self._concatenate_Xb(X, avg_y)

    def solve_LS(self):
        self.G = np.matmul(np.linalg.pinv(self.Xb), self.Yb)
        print("offset values", self.G)

        res = np.mean(np.abs(self.Yb - np.matmul(self.Xb, self.G)))

        print(res)

        self._make_csv()

    def _make_csv(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('force_observer')
        with open(path + '/' + 'imu_offsets.csv', 'wb') as myfile:
            wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
            wr.writerow(self.G)
