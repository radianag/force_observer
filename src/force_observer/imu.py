
import rospy
import numpy as np
import tf
import std_msgs.msg
import geometry_msgs.msg as gm
import math as mt
import tf
import csv

class Imu:
    def __init__(self):
        self.name = "IMU"
        self.sub = rospy.Subscriber('/accel', std_msgs.msg.Int16MultiArray, self.callback)
        self.pub = rospy.Publisher('/accel_calibrated', gm.Accel, queue_size=1)

        self.avg_data_pts = 0

        # Least Squares stuff
        self.Xb = np.zeros([3, 12])
        self.x_array = np.zeros([1, 3])

        self.yb = np.zeros([1, 3])
        self.y_array = np.zeros([1, 3])

        self.a = np.zeros(12)

        self.data = np.zeros(7)
        self.g = np.array([0, 0, -9.81])

        self.avg_cnt = 0
        self.state = 0

        self.offsets = []

        self.new_msg = gm.Accel()

    def setup_calibrate(self, avg_data_points):
        self.avg_data_pts = avg_data_points

    def setup_calibrated_publisher(self, offsets):

        self.offsets = offsets

    def publish_calibrated(self):
        y = self.data[1:4] - self.offsets[-3:]

        A = np.matrix([self.offsets[0:3], self.offsets[3:6], self.offsets[6:9]])
        A = np.transpose(A)
        x = np.matmul(np.linalg.inv(A), y)

        #print(self.offsets[0:3])

        self.new_msg.linear.x = x[0, 0]
        self.new_msg.linear.y = x[0, 1]
        self.new_msg.linear.z = x[0, 2]

        self.pub.publish(self.new_msg)

    def callback(self, msg):
        scale = 8.0 * 9.81 / 32767.0
        self.data = np.array(msg.data, dtype=float)
        self.data[1:] = self.data[1:] * scale
        #print(self.data)

    def set_data(self, R1):
        x = np.matmul(R1[0:3, 0:3], self.g)
    
        if self.avg_cnt == 0:
            self.x_array[0] = x
            self.y_array[0] = np.array([self.data[1], self.data[2], self.data[3]])

        else:
            self.x_array = np.concatenate((self.x_array, x), axis=0)
            self.y_array = np.concatenate((self.y_array, [[self.data[1], self.data[2], self.data[3]]]), axis=0)

        self.avg_cnt += 1

    def make_X(self, x):
        R = [None] * 3
        for i in range(3):
            R[i] = np.eye(3) * x[i]
            if i == 0:
                X = R[i]
            else:
                X = np.concatenate((X, R[i]), axis=1)
        X = np.concatenate((X, np.eye(3)), axis=1)
        return X

    def concatenate_Xb(self, X, y):
        if np.sum(self.Xb) == 0:
            self.Xb = X
            self.Yb = y
        else:
            self.Xb = np.concatenate((self.Xb, X), axis=0)
            self.Yb = np.concatenate((self.Yb, y))
        print("XB", np.shape(self.Xb), np.shape(self.Yb))
        print("Yb", self.Yb)

    def get_avg(self):
        if self.avg_cnt > self.avg_data_pts:
            avg_x = np.sum(self.x_array, axis=0) / self.avg_cnt
            avg_y = np.sum(self.y_array, axis=0) / self.avg_cnt
            # print("avgcnt", self.avg_cnt)
            # print("len", np.shape(self.y_array))

            self.state += 1
            self.avg_cnt = 0
            self.x_array = np.zeros([1, 3])
            self.y_array = np.zeros([1, 3])
            #print(avg_x[0][:])
            X = self.make_X(np.array([avg_x[0,0],avg_x[0,1],avg_x[0,2]]))
            self.concatenate_Xb(X, avg_y)

    def solve_LS(self):
        self.G = np.matmul(np.linalg.pinv(self.Xb), self.Yb)
        print("offset values", self.G)

        res = np.linalg.norm(self.Yb - np.matmul(self.Xb, self.G))

        print(res)

        self.make_csv()

    def make_csv(self):
        with open('imu_offsets.csv', 'wb') as myfile:
            wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
            wr.writerow(self.G)