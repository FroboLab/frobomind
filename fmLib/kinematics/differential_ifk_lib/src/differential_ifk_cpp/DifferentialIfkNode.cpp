/*
 * DifferentialIfkNode.cpp
 *
 *  Created on: Apr 7, 2014
 *      Author: leon
 */
#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "DifferentialIfkNode.hpp"

DifferentialIfkNode::DifferentialIfkNode()
:local_node_handler("~"),global_node_handler()
{
	double wheel_distance;
	std::string twist_sub_topic, left_pub_topic, right_pub_topic;

	// Parse local parameters
	local_node_handler.param<std::string>("twist_sub", twist_sub_topic, "/fmCommand/cmd_vel");
	local_node_handler.param<std::string>("left_pub", left_pub_topic, "/fmSignals/cmd_vel_left");
	local_node_handler.param<std::string>("right_pub", right_pub_topic, "/fmSignals/cmd_vel_right");

	// Parse global parameters
	global_node_handler.param<double>("/diff_steer_wheel_distance",wheel_distance,0.0);
	if(! wheel_distance)
	{
		//if global parameter was not found, look for local
		local_node_handler.param<double>("distance_center_to_wheel",wheel_distance,0.5);
		ROS_WARN("%s: Global parameter 'diff_steer_wheel_distance' was not found - using local instead",ros::this_node::getName().c_str());
	}

	differential.setWheelDistance(wheel_distance);

	// Setup publishers and subscribers
	hl_subscriber = local_node_handler.subscribe<geometry_msgs::TwistStamped> (twist_sub_topic.c_str(), 1, &DifferentialIfkNode::callback, this);
	ll_publisher_left = local_node_handler.advertise<geometry_msgs::TwistStamped> (left_pub_topic.c_str(), 1);
	ll_publisher_right = local_node_handler.advertise<geometry_msgs::TwistStamped> (right_pub_topic.c_str(), 1);
}

DifferentialIfkNode::~DifferentialIfkNode()
{

}

void DifferentialIfkNode::spin(void)
{
	ros::spin();
}

void DifferentialIfkNode::callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	DifferentialIfk::wheel_t velocities = differential.inverse(msg->twist.linear.x, msg->twist.angular.z);

	rtq_command_msg_left.twist.linear.x = velocities.left;
	rtq_command_msg_left.header.stamp = ros::Time::now();
	ll_publisher_left.publish(rtq_command_msg_left);

	rtq_command_msg_right.twist.linear.x = velocities.right;
	rtq_command_msg_right.header.stamp = ros::Time::now();
	ll_publisher_right.publish(rtq_command_msg_right);
}
