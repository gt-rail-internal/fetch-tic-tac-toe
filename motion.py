#!/usr/bin/env python

# simple_disco.py: Move the fetch arm through a simple disco motion
import sys

import tf
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import actionlib
from control_msgs.msg import GripperCommandGoal, GripperCommandAction

from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

robot = None
scene = None

group_name = "arm_with_torso"
move_group_ik = None

# connect to the gripper controller client
client = actionlib.SimpleActionClient('gripper_controller/gripper_action', GripperCommandAction)
moveit_commander.roscpp_initialize("")  # init the MoveIt commander
robot = moveit_commander.RobotCommander()  # init the robot
scene = moveit_commander.PlanningSceneInterface()  # init the planning scene
move_group_ik = moveit_commander.MoveGroupCommander(group_name)  # init the inverse kinematics

def move_arm_joints(joints):
    move_group_joints = MoveGroupInterface("arm_with_torso", "base_link")
    joint_names = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    move_group_joints.moveToJointPosition(joint_names, joints, wait=True)
    return

def get_pose(link="wrist_roll_link"):
    msg = move_group_ik.get_current_pose(end_effector_link=link)

    # Extract position
    pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    # Obtain euler orientation by extracting quaternion and converting
    ort = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

    return pos, ort

# moves the end effector of Fetch to a specified x/y/z location and at a specified a/b/c/w quaternion
def move_arm_ik(x, y, z, a, b, c, w):
    # create the pose goal object
    pose_goal = geometry_msgs.msg.Pose()

    # set the end effector orientation
    pose_goal.orientation.x = a   
    pose_goal.orientation.y = b   
    pose_goal.orientation.z = c   
    pose_goal.orientation.w = w   

    # set the end effector location
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    # send the pose goal
    move_group_ik.set_pose_target(pose_goal)
    plan = move_group_ik.go(wait=True)
    move_group_ik.stop()
    move_group_ik.clear_pose_targets()
    return plan


# sets the gripper width
def move_gripper(w):
    '''
    Function to move gripper. Inputs are
        w = 0: closes the gripper
        w = 1: opens the gripper
    '''

    # ensure connection to the client
    client.wait_for_server()
    print('Client active')

    # create the gripper goal object
    gripper_goal = GripperCommandGoal()
    gripper_goal.command.position = w
    gripper_goal.command.max_effort = 200

    # send the gripper goal
    client.send_goal(gripper_goal)
    
    # wait for the gripper
    client.wait_for_result()  
    return

def go_to_joint(joint):
    # # Torso lift stays fixed
    # joint[0] = 0.37

    # Go to joint position
    move_arm_joints(joint)
    rospy.sleep(0.75)

    return
