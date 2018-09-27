#!/usr/bin/env python

import rospy
import numpy as np
import csv

def callback(msg, offsets, pub):
    scale = 8.0 * 9.81 / 32767.0 
    
    data = np.array(msg.data, dtype=float)
    data[1:] = data[1:] *scale

    y = data[1:4] - offsets[-3:]

    A = np.matrix([offsets[0:3], offsets[3:6], offsets[6:9]])
    x = np.linalg.inv(A)*y

    msg.data = [msg.data[0], x, data[4:]]

    pub.Publish(msg)


if __name__ == '__main__':
	
    name =  'test_offset'
	offsets = np.genfromtxt(name + '.csv', delimiter=',')

	#Settings
    rospy.init_node('Test_node', anonymous=True)
    pub = rospy.Publisher('/accel_calibrated', std_msgs.msg.Int16MultiArray, queue_size=1)
    sub = rospy.Subscriber('/accel', std_msgs.msg.Int16MultiArray, self.callback, offsets, pub)

    rospy.spin()