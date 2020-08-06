#!/usr/bin/env python
 
#import roslib
#import roslib; roslib.load_manifest('smach_ros')
import sys
import copy
import rospy
import math
import tf
import time
import moveit_commander 
import moveit_msgs.msg
import smach
import smach_ros
import geometry_msgs.msg
from smach import CBState
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, JointConstraint, Constraints


# define state Navigate
class Navigate(smach.State):
	def __init__(self,nav_x,nav_y):
		smach.State.__init__(self, outcomes=['Goal_reached', 'Goal_cancelled', 'Goal_aborted'])
		self.nav_x=nav_x
		self.nav_y=nav_y	
	
	def active_cb(self):
		rospy.loginfo("Goal pose being processed")

	def feedback_cb(self,feedback):
		rospy.loginfo("Current location " +str(feedback))
	
	def done_cb(self,status, result):
		self.stat=status
		return self.stat

	
	def execute(self, userdata):
		rospy.loginfo('Starting Navigation')
		
		self.navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.navclient.wait_for_server()

		goal= MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()

		goal.target_pose.pose.position.x = self.nav_x
		goal.target_pose.pose.position.y = self.nav_y
		goal.target_pose.pose.position.z = 0
		goal.target_pose.pose.orientation.x = 0
		goal.target_pose.pose.orientation.y = 0
		goal.target_pose.pose.orientation.z = 0.707 
		goal.target_pose.pose.orientation.w = 0.707 


		self.navclient.send_goal(goal, self.done_cb , self.active_cb, self.feedback_cb)
		self.finished = self.navclient.wait_for_result()
		#print(self.stat)
		
		if not self.finished:
			rospy.loginfo("Action server not available")
		
		elif self.stat == 3:
			return 'Goal_reached'
		
		elif self.stat == 2 or self.stat == 8:
			return 'Goal_cancelled'
		
		elif self.stat == 4:
			return 'Goal_aborted'
		





# define state MoveArm
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['Task1_succeeded','Task1_aborted'])
def MoveArm(user_data):
	rospy.loginfo("Moving Neo's left arm")

	listener = tf.TransformListener()
	start_t = time.time()
	tol = 0.01

	obj = "object_29"
	flag = True
	while not rospy.is_shutdown() and flag is True:
	    bol = listener.frameExists(obj)
	    if bol is True and flag is True:
		trans,rot = listener.lookupTransform("base_footprint",obj,rospy.Time())
		global x,y,z
		x, y, z = trans
		print("X ",x)
		print("Y ",y)
		print("Z ",z)
		flag = False
	    elif bol is False and flag is True: 
		print("Waiting for transform")
	
	global pose_target
	pose_target = geometry_msgs.msg.PoseStamped()
	pose_target.header.frame_id = "base_link"
	pose_target.pose.position.x = round(x,3) - 0.15 
	pose_target.pose.position.y = round(y,3) - 0.017
	pose_target.pose.position.z = round(z,3) - 0.01

	pose_target.pose.orientation.x = 0
	pose_target.pose.orientation.y = 0
	pose_target.pose.orientation.z = 0
	pose_target.pose.orientation.w = 1

	arm_group.set_goal_tolerance(0.007)
	arm_group.set_pose_target(pose_target)
	
	arm_group.set_planning_time(30)
	plan1 = arm_group.plan()
	execute1 = arm_group.go(wait=True)

	if execute1:
		return 'Task1_succeeded'
	else:
		return 'Task1_aborted'
         
# define state Opengripper 
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['Task2_succeeded','Task2_aborted'])
def OpenGripper(user_data):
	rospy.loginfo("Opening gripper")
	
	hand_group.set_named_target("l_open")
	execute2 = hand_group.go(wait=True)
	#hand_group.stop()
	#hand_group.clear_pose_targets()

	if execute2:
		return 'Task2_succeeded'
	else:
		return 'Task2_aborted' 

# define state MoveArmCloser 
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['Task3_succeeded','Task3_aborted'])
def MoveArmCloser(user_data):
	rospy.loginfo("Moving Neo's left arm close to the object")
	
	pose_target.pose.position.x = round(x,3) + 0.065
	arm_group.set_pose_target(pose_target)
	#plan3 = arm_group.plan()
	execute3 = arm_group.go(wait=True)

	if execute3:
		return 'Task3_succeeded'
	else:
		return 'Task3_aborted' 

# define state CloseGripper
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['Task4_succeeded','Task4_aborted'])
def CloseGripper(user_data):
	rospy.loginfo("Closing gripper")
	
	group_variable_values = hand_group.get_current_joint_values()
	group_variable_values[0] = 0.045
	group_variable_values[1] = 0.045
	hand_group.set_joint_value_target(group_variable_values)
	execute4 = hand_group.go(wait=True)

	if execute4:
		return 'Task4_succeeded'
	else:
		return 'Task4_aborted'

# define state RetractArm after picking up the object
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['Task5_succeeded','Task5_aborted'])
def RetractArm(user_data):
	rospy.loginfo("Retracting Neo's arm")
	
	pose_target.pose.position.z = round(z,3) + 0.1
	arm_group.set_pose_target(pose_target)
	execute5 = arm_group.go(wait=True)

	#rospy.sleep(1)

	arm_group.set_named_target("l_pose1")
	execute6 = arm_group.go(wait=True)

	if execute5 and execute6:
		return 'Task5_succeeded'
	else:
		return 'Task5_aborted'

# define state PlaceObject
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['Task6_succeeded','Task6_aborted'])
def PlaceObject(user_data):
	rospy.loginfo("Placing Object")
	
	pose_target.pose.position.x = 0.847
	pose_target.pose.position.y = 0.172
	pose_target.pose.position.z = 0.899

	pose_target.pose.orientation.x = 0
	pose_target.pose.orientation.y = 0
	pose_target.pose.orientation.z = 0
	pose_target.pose.orientation.w = 1

	arm_group.set_goal_tolerance(0.007)
	arm_group.set_pose_target(pose_target)
	
	arm_group.set_planning_time(30)
	plan7 = arm_group.plan()
	execute7 = arm_group.go(wait=True)

	if execute7:
		return 'Task6_succeeded'
	else:
		return 'Task6_aborted'

# define state RetractArm after placing the object
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['Task7_succeeded','Task7_aborted'])
def RetractArm2(user_data):
	rospy.loginfo("Retracting Neo's arm")
	
	pose_target.pose.position.x = round(x,3) - 0.17
	arm_group.set_pose_target(pose_target)
	execute8 = arm_group.go(wait=True)

	rospy.sleep(1)

	rospy.loginfo("Closing gripper")
	
	hand_group.set_named_target("l_close")
	execute9 = hand_group.go(wait=True)
	
	rospy.sleep(1)

	arm_group.set_named_target("l_idle")
	execute10 = arm_group.go(wait=True)

	if execute8 and execute9 and execute10:
		return 'Task7_succeeded'
	else:
		return 'Task7_aborted'


 

def main():
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('neo_state_machine',anonymous=True)
	global robot, scene, arm_group, hand_group, display_trajectory_publisher
	robot = moveit_commander.RobotCommander()  
	scene = moveit_commander.PlanningSceneInterface()

	arm_group = moveit_commander.MoveGroupCommander("l_arm")
	hand_group= moveit_commander.MoveGroupCommander("l_gripper")


	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path' , moveit_msgs.msg.DisplayTrajectory,queue_size=20)


	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['TASK_FINISHED','TASK_ABORTED'])

	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()
	# Open the container
	with sm:
	# Add states to the container
		smach.StateMachine.add('NAVIGATE', Navigate(4.012,6.978), transitions={'Goal_reached':'MOVE_ARM', 'Goal_cancelled':'TASK_ABORTED', 'Goal_aborted':'TASK_ABORTED'}) 
		
		smach.StateMachine.add('MOVE_ARM', CBState(MoveArm), transitions={'Task1_succeeded':'OPEN_GRIPPER','Task1_aborted':'TASK_ABORTED'}) 
 		
		smach.StateMachine.add('OPEN_GRIPPER', CBState(OpenGripper), transitions={'Task2_succeeded':'MOVE_ARM_CLOSE_TO_OBJECT','Task2_aborted':'TASK_ABORTED'}) 

		smach.StateMachine.add('MOVE_ARM_CLOSE_TO_OBJECT', CBState(MoveArmCloser), transitions={'Task3_succeeded':'CLOSE_GRIPPER', 'Task3_aborted':'TASK_ABORTED'}) 

		smach.StateMachine.add('CLOSE_GRIPPER', CBState(CloseGripper), transitions={'Task4_succeeded':'RETRACT_ARM', 'Task4_aborted':'TASK_ABORTED'})		
		 
		smach.StateMachine.add('RETRACT_ARM', CBState(RetractArm), transitions={'Task5_succeeded':'NAVIGATE2', 'Task5_aborted':'TASK_ABORTED'})

		smach.StateMachine.add('NAVIGATE2', Navigate(-1.993,7.011), transitions={'Goal_reached':'MOVE_ARM_TO_PLACE_OBJECT', 'Goal_cancelled':'TASK_ABORTED', 'Goal_aborted':'TASK_ABORTED'})

		smach.StateMachine.add('MOVE_ARM_TO_PLACE_OBJECT', CBState(PlaceObject), transitions={'Task6_succeeded':'OPEN_GRIPPER2','Task6_aborted':'TASK_ABORTED'}) 

		smach.StateMachine.add('OPEN_GRIPPER2', CBState(OpenGripper), transitions={'Task2_succeeded':'RETRACT_ARM2','Task2_aborted':'TASK_ABORTED'})

		smach.StateMachine.add('RETRACT_ARM2', CBState(RetractArm2), transitions={'Task7_succeeded':'TASK_FINISHED', 'Task7_aborted':'TASK_ABORTED'})


		
	# Execute SMACH plan
	outcome = sm.execute()

	rospy.spin()
	sis.stop()

 
if __name__ == '__main__':
	main()
