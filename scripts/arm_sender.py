#! /usr/bin/env python

import rospy
from arm_sender.OpenManipulator import OpenManipulator


if __name__ == '__main__':
    try:
        rospy.init_node('arm_sender')
        arm = OpenManipulator()
        rospy.loginfo("send1st")
        arm.set_arm_goal([0, 0, 0, 0], rospy.Duration(2))
        arm.set_arm_goal([0, -0.0, -0.2, 0.0], rospy.Duration(3))
        arm.set_arm_goal([0, -0.0, -0.2, 0.2], rospy.Duration(4))
        # arm.set_arm_goal([0, -0.0, -0.4, 0.2], rospy.Duration(7))
        # arm.set_arm_goal([0, -0.0, -0.4, 0.4], rospy.Duration(8))
        # arm.set_arm_goal([0, -0.0, -0.6, 0.4], rospy.Duration(9))
        arm.send_arm_goal()
        rospy.loginfo("end1st")
        rospy.sleep(5)

    except rospy.ROSInterruptException:
        pass
