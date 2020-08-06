/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>


class NeoTeleop {
public:
	NeoTeleop();

	void send_cmd();

protected:
	void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);

private:
	ros::NodeHandle nh;

	ros::Publisher vel_pub;
	ros::Subscriber joy_sub;
	geometry_msgs::Twist cmd_vel;

	double linear_scale_x = 0;
	double linear_scale_y = 0;
	double angular_scale_z = 0;
	double smooth_factor = 1;
	double joy_timeout = 0;
	int axis_linear_x = -1;
	int axis_linear_y = -1;
	int axis_angular_z = -1;
	int deadman_button = -1;

	ros::Time last_joy_time;
	double joy_command_x = 0;
	double joy_command_y = 0;
	double joy_command_z = 0;

	bool is_active = false;
	bool is_deadman_pressed = false;

};

NeoTeleop::NeoTeleop()
{
	nh.param("scale_linear_x", linear_scale_x, 0.4);
	nh.param("scale_linear_y", linear_scale_y, 0.4);
	nh.param("scale_angular_z", angular_scale_z, 0.6);
	nh.param("axis_linear_x", axis_linear_x, 1);
	nh.param("axis_linear_y", axis_linear_y, 0);
	nh.param("axis_angular_z", axis_angular_z, 2);
	nh.param("smooth_factor", smooth_factor, 0.2);
	nh.param("deadman_button", deadman_button, 5);
	nh.param("joy_timeout", joy_timeout, 1.);

	vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &NeoTeleop::joy_callback, this);
}

void NeoTeleop::joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
	last_joy_time = ros::Time::now();

	if(deadman_button >= 0 && deadman_button < joy->buttons.size()) {
		is_deadman_pressed = (bool)joy->buttons[deadman_button];
	} else {
		is_deadman_pressed = false;
	}
	if(is_deadman_pressed) {
		is_active = true;
	}

	if(axis_linear_x >= 0 && axis_linear_x < joy->axes.size()) {
		joy_command_x = linear_scale_x * joy->axes[axis_linear_x];
	}
	if(axis_linear_y >= 0 && axis_linear_y < joy->axes.size()) {
		joy_command_y = linear_scale_y * joy->axes[axis_linear_y];
	}
	if(axis_angular_z >= 0 && axis_angular_z < joy->axes.size()) {
		joy_command_z = angular_scale_z * joy->axes[axis_angular_z];
	}
}

void NeoTeleop::send_cmd()
{
	if(is_deadman_pressed)
	{
		// smooth inputs
		cmd_vel.linear.x = joy_command_x * smooth_factor + cmd_vel.linear.x * (1 - smooth_factor);
		cmd_vel.linear.y = joy_command_y * smooth_factor + cmd_vel.linear.y * (1 - smooth_factor);
		cmd_vel.angular.z = joy_command_z * smooth_factor + cmd_vel.angular.z * (1 - smooth_factor);

		// publish
		vel_pub.publish(cmd_vel);
	}
	else if(is_active)
	{
		if((ros::Time::now() - last_joy_time).toSec() > joy_timeout)
		{
			cmd_vel = geometry_msgs::Twist();		// set to all zero
			is_active = false;
		}
		else {
			// smooth towards zero
			cmd_vel.linear.x = cmd_vel.linear.x * (1 - smooth_factor);
			cmd_vel.linear.y = cmd_vel.linear.y * (1 - smooth_factor);
			cmd_vel.angular.z = cmd_vel.angular.z * (1 - smooth_factor);
		}

		// publish
		vel_pub.publish(cmd_vel);
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "neo_teleop");

	NeoTeleop node;

	ros::Rate loop_rate(50); // Hz

	while(ros::ok())
	{
		node.send_cmd();

		loop_rate.sleep();

		ros::spinOnce();
	}

	return 0;
}
