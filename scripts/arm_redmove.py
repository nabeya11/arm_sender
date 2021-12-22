#! /usr/bin/env python

import rospy
from arm_sender.OpenManipulator import OpenManipulator
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class Arm_Follow:
    def __init__(self):
        self.object_sub = rospy.Subscriber('object_odom', Twist, self.callback)
        self.joint_sub = rospy.Subscriber('joint_states', JointState, self.joint_callback)
        self.arm = OpenManipulator()
        self.joint_states=[]
        self.pitch = 0
        self.yaw = 0
    def joint_callback(self, data):
        self.joint_states = data.position
    def callback(self, data):
        rospy.loginfo(data.linear)

        kz = 0.0005
        ky = 0.003
        kx = 0.003
        self.depth = -kz*(data.linear.z - 200)
        self.pitch = self.joint_states[5] + ky*(data.linear.y - 240)
        self.yaw   = self.joint_states[2] - kx*(data.linear.x - 320)

        self.yaw = max(self.yaw, -1.57)
        self.yaw = min(self.yaw, 1.57)

        self.pitch = max(self.pitch, -1.57)
        self.pitch = min(self.pitch, 1.57)

        self.arm.set_arm_goal([self.yaw, 0, 0, self.pitch], rospy.Duration(1))
        self.arm.send_arm_goal()


if __name__ == '__main__':
    rospy.init_node('arm_follow_object')
    arm=Arm_Follow()
    try:
        rospy.spin()
        #TODO Waffle never stop even when this program is killed...
    except rospy.ROSInterruptException:
        pass
