#! /usr/bin/env python

import sys
import copy
import rospy
import math
import tf
import time
import moveit_commander 
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, JointConstraint, Constraints

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('python_node',anonymous=True)

robot = moveit_commander.RobotCommander()  
scene = moveit_commander.PlanningSceneInterface()

arm_group = moveit_commander.MoveGroupCommander("l_arm")
hand_group= moveit_commander.MoveGroupCommander("l_gripper")


display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path' , moveit_msgs.msg.DisplayTrajectory,queue_size=20)


listener = tf.TransformListener()
start_t = time.time()
tol = 0.01

obj = "object_29"
flag = True
while not rospy.is_shutdown() and flag is True:
    bol = listener.frameExists(obj)
    if bol is True and flag is True:
        trans,rot = listener.lookupTransform("base_footprint",obj,rospy.Time())
        x,y,z = trans
        print(x)
        flag = False
    elif bol is False and flag is True: 
        print("error no such transform")

pose_target = geometry_msgs.msg.PoseStamped()
pose_target.header.frame_id = "base_link"
pose_target.pose.position.x = round(x,3) - 0.15 #0.70497235  #0.686035203183#0.815497296598
pose_target.pose.position.y = round(y,3)#0.05076018 #0.081532898455#-0.00681731667033
pose_target.pose.position.z = round(z,3) - 0.01#0.89241191#0.988392256467#1.21581034818#0.966

pose_target.pose.orientation.x = 0#-0.707310386272#0.0190057538173#-0.00413393705118#0.00289348192451
pose_target.pose.orientation.y = 0#0.000244863833822#-0.00127320518284#-0.000217395710805#0.0145272904403
pose_target.pose.orientation.z = 0#-0.00125703791881#-0.000114365375937#-0.000569282690604#3.82616797725e-05
pose_target.pose.orientation.w = 1#0.706901957396#0.9998#0.9989

arm_group.set_goal_tolerance(0.006)
arm_group.set_pose_target(pose_target)
#group.set_path_constraints(upright_constraints)
arm_group.set_planning_time(50)
plan = arm_group.plan()
arm_group.go(wait=True)
#arm_group.stop()
#arm_group.clear_pose_targets()
'''
group_variable_values = hand_group.get_current_joint_values()
group_variable_values[0] = 0.07
group_variable_values[1] = 0.07
hand_group.set_joint_value_target(group_variable_values)
hand_group.go(wait=True)
'''
hand_group.set_named_target("l_open")
hand_group.go(wait=True)
#hand_group.stop()
#hand_group.clear_pose_targets()

pose_target.pose.position.x = round(x,3) + 0.065
arm_group.set_pose_target(pose_target)
plan4 = arm_group.plan()
arm_group.go(wait=True)

rospy.sleep(2)

group_variable_values = hand_group.get_current_joint_values()
group_variable_values[0] = 0.045
group_variable_values[1] = 0.045
hand_group.set_joint_value_target(group_variable_values)
hand_group.go(wait=True)

pose_target.pose.position.z = round(z,3) + 0.1
arm_group.set_pose_target(pose_target)
arm_group.go(wait=True)

moveit_commander.roscpp_shutdown()

