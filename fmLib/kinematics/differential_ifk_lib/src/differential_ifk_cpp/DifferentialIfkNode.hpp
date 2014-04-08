/*
 * DifferentialIfkNode.hpp
 *
 *  Created on: Apr 7, 2014
 *      Author: leon
 */

#ifndef DIFFERENTIALIFKNODE_HPP_
#define DIFFERENTIALIFKNODE_HPP_

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include "differential_ifk_lib/DifferentialIfk.hpp"

class DifferentialIfkNode
{
private:
	ros::NodeHandle local_node_handler,global_node_handler;

	geometry_msgs::TwistStamped rtq_command_msg_left;
	geometry_msgs::TwistStamped rtq_command_msg_right;

	ros::Subscriber hl_subscriber;
	ros::Publisher ll_publisher_left;
	ros::Publisher ll_publisher_right;

	DifferentialIfk differential;

public:
	DifferentialIfkNode();
	virtual ~DifferentialIfkNode();

	void spin(void);
	void callback(const geometry_msgs::TwistStamped::ConstPtr&);
};

#endif /* DIFFERENTIALIFKNODE_HPP_ */
