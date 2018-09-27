#!/usr/bin/env python

import rospy
import numpy as np
import tf
import csv
import dvrk

import std_msgs.msg
import math as mt
import tf

class Imu:
    def __init__(self, avg_data_points):
        self.name = "IMU"
        self. sub = rospy.Subscriber('/accel', std_msgs.msg.Int16MultiArray, self.callback)

        self.avg_data_pts = avg_data_points

        #Least Squares stuff
        self.Xb = np.zeros([3, 12])
        self.x_array = np.zeros([1, 3])

        self.yb = np .zeros([1,3])
        self.y_array = np .zeros([1,3])

        self.a = np.zeros(12)
   
        self.data = np.zeros(7)
        self.g = np.array([0, 0, -9.81])

        self.avg_cnt = 0
        self.state = 0

    def callback(self, msg):
        scale = 8.0 * 9.81 / 32767.0 
        self.data = np.array(msg.data, dtype=float)
        self.data[1:] = self.data[1:] *scale

    def set_data(self, R1):
        x = np.matmul(R1[0:3, 0:3], self.g)
        if self.avg_cnt == 0:
            self.x_array[0] = x
            self.y_array[0] = np.array([self.data[1], self.data[2], self.data[3]])
          
        else:
            self.x_array = np.concatenate((self.x_array, [x]), axis=0)
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
        X = np.concatenate((X,np.eye(3)), axis=1)
        return X

    def concatenate_Xb(self, X, y):
        if np.sum(self.Xb) == 0:
            self.Xb = X
            self.Yb = y
        else:
            self.Xb = np.concatenate((self.Xb, X), axis=0)
            self.Yb = np.concatenate((self.Yb, y))
        print("XB" , np.shape(self.Xb), np.shape(self.Yb))
        print("Yb", self.Yb)

    def get_avg(self):
        if self.avg_cnt > self.avg_data_pts:
            avg_x = np.sum(self.x_array, axis=0)/self.avg_cnt
            avg_y = np.sum(self.y_array, axis=0)/self.avg_cnt
            #print("avgcnt", self.avg_cnt)
            #print("len", np.shape(self.y_array))

            self.state += 1
            self.avg_cnt = 0
            self.x_array = np.zeros([1, 3])
            self.y_array = np.zeros([1, 3])
            X = self.make_X(avg_x)
            self.concatenate_Xb(X, avg_y)

    def solve_LS(self):
        self.G = np.matmul(np.linalg.pinv(self.Xb),self.Yb)
        print("offset values", self.G)
        self.make_csv()

    def make_csv(self):
        with open('imu_offsets.csv', 'wb') as myfile:
            wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
            wr.writerow(self.G)

if __name__ == '__main__':
    rospy.init_node('calibration', anonymous=True)

    #Settings
    ctrl_rate = 100       # [Hz]
    stabil_ts = 2 #[sec]
    
    q1_num_pts = 4
    q2_num_pts = 4

    limit = [(-1.54, 1.54), (-0.5, 0.5)]
    q3_pos = 0.05

    r = rospy.Rate(ctrl_rate)

    traj_q1 = np.linspace(limit[0][0], limit[0][1], num=q1_num_pts)
    traj_q2 = np.linspace(limit[1][0], limit[1][1], num=q2_num_pts)

    num_points = q1_num_pts*q2_num_pts

    p = dvrk.psm('PSM1')
    p.home()
    imu = Imu(ctrl_rate*stabil_ts)

    yaw = 1
    pitch = 1.54

    state1 = np.linspace(0,q1_num_pts-1,q1_num_pts, dtype=int)
    state2 = np.linspace(0,q2_num_pts-1,q2_num_pts, dtype=int)

    flat_state= [(0,0)]* num_points
    for i in range(q1_num_pts):
        for j in range(q2_num_pts):
            flat_state[(i)*q2_num_pts+j] = (state1[i], state2[j])
    print(traj_q1)

    a = [0, 0]
    while (imu.state < num_points ) and not rospy.is_shutdown():

        if imu.avg_cnt == 0:
            p.move_joint_some(np.array([traj_q1[flat_state[imu.state][0]], traj_q2[flat_state[imu.state][1]], q3_pos]),np.array([0, 1, 2]))
            rospy.sleep(5)
        a = p.get_current_joint_position()[0:2]
        yaw = a[0]
        pitch = a[1]
        R1 = tf.transformations.euler_matrix(yaw, pitch, 0, 'syxz')
        imu.set_data(R1)
        imu.get_avg()
        r.sleep()

    imu.solve_LS()

    rospy.spin()



    #
    # # Names: (PSM NAME, ROTATION RIGID BODY NUM, RCM RIGID BODY NUM)
    # names = [('PSM2', 3, 4), ('PSM1', 1, 2)]
    # testname = 'one'
    #
    # calib = Calibrate(names)
    # calib.run_test()
    #
    # rospack = rospkg.RosPack()
    # path = rospack.get_path('dvrk_calibrate')
    # folder = '/data'
    # total = path + folder
    #
    # calib.calculate_transform(False)
    #
    # bpost, rotations = calib.get_results()
    # print("Bpost is:", bpost)
    # print("Rotations is:", rotations)