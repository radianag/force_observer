#!/usr/bin/env python

import rospy
import dynamixel_workbench_msgs.msg as dm
import dynamixel_workbench_msgs.srv as dmsrv

from force_observer.utils import *

class DynamixelController:
    def __init__(self, state_topic='/dynamixel_state', srv_name='/wheel_command'):
        self.state = rospy.Subscriber(state_topic, dm.DynamixelState, self.callback_state)

        self.srv_name = srv_name
        rospy.wait_for_service(self.srv_name)
        self.joint_vel = 0

    def callback_state(self, msg):
        self.joint_vel = msg.present_velocity

    def publish_velocity(self, set_velocity):
        for i in range(3):
            try:
                wheel_command = rospy.ServiceProxy(self.srv_name, dmsrv.WheelCommand)
                resp1 = wheel_command(set_velocity, 0)
                return resp1
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e

    def return_velocity(self):
        return self.joint_vel

