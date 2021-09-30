#! /usr/bin/env python

from actionlib import action_client
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class OpenManipulator:
    def __init__(self):
        self.arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.gripper_client = actionlib.SimpleActionClient('gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    def send_arm_goal(self, arm_goal):
        self.arm_client.wait_for_server()
        arm_traject = JointTrajectory()
        arm_traject.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        arm_traject.points.append(JointTrajectoryPoint())
        arm_traject.points[0].positions = arm_goal
        arm_traject.points[0].time_from_start = rospy.Duration(3)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = arm_traject
        goal.goal_time_tolerance = rospy.Duration(0)
        self.arm_client.send_goal(goal)
        # self.arm_client.wait_for_result()

    def send_gripper_goal(self, gripper_goal):
        self.gripper_client.wait_for_server()
        gripper_traject = JointTrajectory()
        gripper_traject.joint_names = ['gripper']
        gripper_traject.points.append(JointTrajectoryPoint())
        gripper_traject.points[0].positions = [gripper_goal]
        gripper_traject.points[0].time_from_start = rospy.Duration(3)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = gripper_traject
        goal.goal_time_tolerance = rospy.Duration(0)
        self.gripper_client.send_goal(goal)
        # self.gripper_client.wait_for_result()

if __name__ == '__main__':
    try:
        rospy.init_node('arm_sender')
        arm = OpenManipulator()
        rospy.loginfo("send1st")
        arm.send_arm_goal([0, 0, 0, 0])
        arm.send_gripper_goal(-0.02)
        rospy.loginfo("end1st")
        rospy.sleep(1)
        rospy.loginfo("send2nd")
        arm.send_arm_goal([0.7, 0, 0, 0])
        arm.send_gripper_goal(0.0)
        rospy.loginfo("end2nd")
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        pass
