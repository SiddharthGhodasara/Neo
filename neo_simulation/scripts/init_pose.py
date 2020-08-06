#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseCovarianceStamped


rospy.init_node('set_init_pose',anonymous=True)
pub = rospy.Publisher('/initialpose' PoseCovarianceStamped, queue_size=1)

initpose_msg = PoseCovarianceStamped()
initpose_msg.header.frame_id = "map"
initpose_msg.pose.pose.position.x = 
initpose_msg.pose.pose.position.y =
initpose_msg.pose.pose.position.z =
initpose_msg.pose.pose.orientation.x =
initpose_msg.pose.pose.orientation.y =
initpose_msg.pose.pose.orientation.z =
initpose_msg.pose.pose.orientation.w =

rospy.sleep(1)

rospy.loginfo("Setting inital pose")
pub.publish(initpose_msg)
rospy.loginfo("Successfully set inital pose")
