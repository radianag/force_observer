#!/usr/bin/env python

import rospy
import numpy as np
import std_msgs.msg
import csv
import rospkg

class SolverLsAb:
    def __init__(self):
        self.avg_data_pts = 0

        # Least Squares stuff
        self.Xb = np.zeros([3, 12])
        self.x_array = np.zeros([1, 3])

        self.yb = np.zeros([1, 3])
        self.y_array = np.zeros([1, 3])

        self.g = np.array([0, 0, -9.81])

        self.avg_cnt = 0
        self.cur_num_data = 0

    def _concatenate_Xb_yb(self, X, y):
        if np.sum(self.Xb) == 0:
            self.Xb = X
            self.Yb = y
        else:
            self.Xb = np.concatenate((self.Xb, X), axis=0)
            self.Yb = np.concatenate((self.Yb, y))
        print("Xb shape, Yb shape", np.shape(self.Xb), np.shape(self.Yb))

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

    def get_avg_after_num(self):
        # Get the average after set_data many times
        avg_x = np.sum(self.x_array, axis=0) / self.avg_cnt
        avg_y = np.sum(self.y_array, axis=0) / self.avg_cnt
        #print(avg_x)

        self.cur_num_data += 1
        self.avg_cnt = 0
        self.x_array = np.zeros([1, 3])
        self.y_array = np.zeros([1, 3])

        #X = self._make_X(np.array([avg_x[0, 0], avg_x[0, 1], avg_x[0, 2]]))
        X = self._make_X(np.array([avg_x[0], avg_x[1], avg_x[2]]))
        self._concatenate_Xb_yb(X, avg_y)

    def make_csv(self, offsets_name):
        rospack = rospkg.RosPack()
        path = rospack.get_path('force_observer')
        with open(path + '/' + offsets_name + '.csv', 'wb') as myfile:
            wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
            wr.writerow(self.Ab_array)

    def set_data(self, x, y):
        #x and y is a list or np array, get first 3
        #print('setting x data', x)
        #print('setting y data', y)
        if self.avg_cnt == 0:
            self.x_array[0] = np.array([x[0], x[1], x[2]])
            self.y_array[0] = np.array([y[0], y[1], y[2]])
        else:
            self.x_array = np.concatenate((self.x_array, [[x[0], x[1], x[2]]]), axis=0)
            self.y_array = np.concatenate((self.y_array, [[y[0], y[1], y[2]]]), axis=0)
        self.avg_cnt += 1

        #print(self.x_array)

    def solve_LS(self):
        self.Ab_array = np.matmul(np.linalg.pinv(self.Xb), self.Yb)
        print("offset values", self.Ab_array)
        res = np.mean(np.abs(self.Yb - np.matmul(self.Xb, self.Ab_array)))
        print(res)

