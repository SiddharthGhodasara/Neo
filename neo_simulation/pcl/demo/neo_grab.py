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
rospy.init_node('move_python_node',anonymous=True)

robot = moveit_commander.RobotCommander()  
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("left_arm")
#group1 = moveit_commander.MoveGroupCommander("gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path' , moveit_msgs.msg.DisplayTrajectory,queue_size=20)
listener = tf.TransformListener()
start_t = time.time()
tol = 0.01


#group.set_named_target("idle")
#plan1 = group.go(wait=True)
#rospy.sleep(2)
#rospy.sleep(2)
obj = "object_26"
#flag = True
while not rospy.is_shutdown():
    bol = listener.frameExists(obj)
    if bol is True:
        trans,rot = listener.lookupTransform("base_footprint",obj,rospy.Time())
        x,y,z = trans
        curr_x = group.get_current_pose().pose.position.x
        curr_y = group.get_current_pose().pose.position.y
        err_x = abs(x-curr_x)
        err_y = abs(y-curr_y)
        print(err_x)
        if err_x >= 0.100 or err_y>=0.100:
            print("in the loop")
            pose_target = geometry_msgs.msg.PoseStamped()
	        pose_target.header.frame_id = "base_link"
            pose_target.pose.position.x = round(x,2) 
            pose_target.pose.position.y = round(y,2) 
            pose_target.pose.position.z = round(z,2)  
            pose_target.pose.orientation.x = 0.503554079155
	        pose_target.pose.orientation.y = 0.504745118071
	        pose_target.pose.orientation.z = 0.496422192237
	        pose_target.pose.orientation.w =  0.495207696027
            group.set_goal_tolerance(tol)
            group.set_pose_target(pose_target)
            group.set_num_planning_attempts(40)
	        group.set_planning_time(50)
            plan2 = group.go(wait=True)
        else:
            print("too close")
            
    elif bol is False: 
        print("error no such transform")

rospy.sleep(3)
moveit_commander.roscpp_shutdown()
