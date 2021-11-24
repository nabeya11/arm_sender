#! /usr/bin/env python

import rospy
from arm_sender.OpenManipulator import OpenManipulator

if __name__ == '__main__':
    try:
        rospy.init_node('arm_sender')
        arm = OpenManipulator()
        rospy.loginfo("send1st")
        arm.set_arm_goal([0, -1.8, 1.2, 0.3], rospy.Duration(3))
        arm.send_arm_goal()
        arm.send_gripper_goal(0)
        rospy.loginfo("end1st")
        rospy.sleep(5)

    except rospy.ROSInterruptException:
        pass
