/*
 * pathExecutor.cpp
 *
 *  Created on: Jan 1, 2001
 *      Author: soeni05
 */

#include "rabbitPlanner.h"
#include <geometry_msgs/PoseStamped.h>
#include <line_executer/follow_pathAction.h>
#include <line_executer/rabbitPlannerConfig.h>
#include <dynamic_reconfigure/server.h>
rabbitPlanner * rabbit = NULL;

void reconfig_callback(line_executer::rabbitPlannerConfig &config, uint32_t level)
{
	rabbit->setParams(config.Rabbit_type,config.Distance_scale,config.Angle_scale,config.Delta_rabbit,config.Delta_waypoint);

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "rabbitPlanner");
	ros::NodeHandle nh("~");
	ros::NodeHandle n;

	std::string filepath,path_pub_topic;

	rabbit = new rabbitPlanner("follow_path");

	nh.param<std::string>("path_publisher_topic",path_pub_topic,"/fmExecutors/path");
	//base_projected_to_A_B / deltaRabbit = rabbit
	nh.param<double>("deltaRabbit", rabbit->deltaRabbit, 2);
	//Waypoint threshold (0 = change waypoint when rabbit > B has been passed ; 1 = change waypoint when rabbit >= B-1 has been passed)
	nh.param<double>("deltaWaypoint", rabbit->deltaWaypoint, 0);
	nh.param<double>("angle_scale", rabbit->angle_scale, 1);
	nh.param<double>("distance_scale", rabbit->distance_scale, 1);

	nh.param<int>("rabbit_type", rabbit->rabbit_type, 0);

	nh.param<std::string>("odometry_frame", rabbit->odom_frame, "odom");
	nh.param<std::string>("vehicle_frame", rabbit->vehicle_frame, "base_link");
	nh.param<std::string>("rabbit_frame", rabbit->rabbit_frame, "rabbit");

	// latched topic so new subscriber get the most recent path
	rabbit->path_publisher = nh.advertise<nav_msgs::Path>(path_pub_topic.c_str(),5,true);

	dynamic_reconfigure::Server<line_executer::rabbitPlannerConfig> server;

	dynamic_reconfigure::Server<line_executer::rabbitPlannerConfig>::CallbackType cb;

	cb = boost::bind(&reconfig_callback, _1, _2);
	server.setCallback(cb);

	ros::spin();

	return 0;

}
