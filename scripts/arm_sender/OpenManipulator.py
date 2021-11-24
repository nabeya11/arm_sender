import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class OpenManipulator:
    def __init__(self):
        self.arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.gripper_client = actionlib.SimpleActionClient('gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.arm_traject = JointTrajectory()
        self.arm_traject.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.goal = FollowJointTrajectoryGoal()
        self.goal.goal_time_tolerance = rospy.Duration(0)

    def set_arm_goal(self, arm_goal, time):
        self.arm_traject.points.append(JointTrajectoryPoint())
        self.arm_traject.points[-1].positions = arm_goal
        self.arm_traject.points[-1].time_from_start = time

    def send_arm_goal(self):
        self.arm_client.wait_for_server()

        self.goal.trajectory = self.arm_traject
        self.arm_client.send_goal(self.goal)
        # self.arm_client.wait_for_result()
        self.arm_traject.points=[]

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
