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
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, JointConstraint, Constraints
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_python_node',anonymous=True)

robot = moveit_commander.RobotCommander()  
scene = moveit_commander.PlanningSceneInterface()

arm_group = moveit_commander.MoveGroupCommander("l_arm")
hand_group= moveit_commander.MoveGroupCommander("l_gripper")


display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path' , moveit_msgs.msg.DisplayTrajectory,queue_size=20)


'''

# ----- NAVIGATION STACK ------#
def active_cb(extra):
	rospy.loginfo("Goal pose being processed")

def feedback_cb(feedback):
	rospy.loginfo("Current location" +str(feedback))

def done_cb(status, result):
	if status == 3:
		rospy.loginfo("Goal reached")

	if status == 2 or status == 8:
		rospy.loginfo("Goal cancelled")

	if status == 4:
		rospy.loginfo("Goal aborted")


navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
navclient.wait_for_server()

goal= MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()

goal.target_pose.pose.position.x = 4.012#first table
goal.target_pose.pose.position.y = 6.978
goal.target_pose.pose.position.z = 0
goal.target_pose.pose.orientation.x = 0
goal.target_pose.pose.orientation.y = 0
goal.target_pose.pose.orientation.z = 0.707 #0.700
goal.target_pose.pose.orientation.w = 0.707 #0.714
#second table -1.993 7.011 0.000 0.000, 0.000, 0.705, 0.70

navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
finished = navclient.wait_for_result()

if not finished:
	rospy.loginfo("Action server not available")
else:
	rospy.loginfo(navclient.get_result())
'''
# ------- MOVEIT -------#

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
        print("X",x)
	print("Y",y)
        flag = False
    elif bol is False and flag is True: 
        print("error no such transform")

pose_target = geometry_msgs.msg.PoseStamped()
pose_target.header.frame_id = "base_link"
pose_target.pose.position.x = round(x,3) - 0.15 #0.70497235  #0.686035203183#0.815497296598
pose_target.pose.position.y = round(y,3)#0.05076018 #0.081532898455#-0.00681731667033
pose_target.pose.position.z = round(z,3) - 0.007#0.89241191#0.988392256467#1.21581034818#0.966

pose_target.pose.orientation.x = 0#-0.707310386272#0.0190057538173#-0.00413393705118#0.00289348192451
pose_target.pose.orientation.y = 0#0.000244863833822#-0.00127320518284#-0.000217395710805#0.0145272904403
pose_target.pose.orientation.z = 0#-0.00125703791881#-0.000114365375937#-0.000569282690604#3.82616797725e-05
pose_target.pose.orientation.w = 1#0.706901957396#0.9998#0.9989

arm_group.set_goal_tolerance(0.007)
arm_group.set_pose_target(pose_target)
#group.set_path_constraints(upright_constraints)
arm_group.set_planning_time(20)
plan = arm_group.plan()
plan1 =arm_group.go(wait=True)
#print("MOVEIT_OUTPUT",plan1)
#arm_group.stop()
#arm_group.clear_pose_targets()
'''
group_variable_values = hand_group.get_current_joint_values()
group_variable_values[0] = 0.07
group_variable_values[1] = 0.07
hand_group.set_joint_value_target(group_variable_values)
hand_group.go(wait=True)
'''
rospy.sleep(2)
hand_group.set_named_target("l_open")
hand_group.go(wait=True)
#hand_group.stop()
#hand_group.clear_pose_targets()

rospy.sleep(2)

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

rospy.sleep(2)

pose_target.pose.position.z = round(z,3) + 0.1
arm_group.set_pose_target(pose_target)
arm_group.go(wait=True)



'''
goal1= MoveBaseGoal()
goal1.target_pose.header.frame_id = "map"
goal1.target_pose.header.stamp = rospy.Time.now()

goal1.target_pose.pose.position.x = -1.993#first table
goal1.target_pose.pose.position.y = 7.011
goal1.target_pose.pose.position.z = 0.000
goal1.target_pose.pose.orientation.x = 0.000
goal1.target_pose.pose.orientation.y = 0.000
goal1.target_pose.pose.orientation.z = 0.705
goal1.target_pose.pose.orientation.w = 0.7

#second table -1.993 7.011 0.000 0.000, 0.000, 0.705, 0.70

navclient.send_goal(goal1, done_cb, active_cb, feedback_cb)
finished1 = navclient.wait_for_result()

if not finished:
	rospy.loginfo("Action server not available")
else:
	rospy.loginfo(navclient.get_result())
'''
moveit_commander.roscpp_shutdown()
