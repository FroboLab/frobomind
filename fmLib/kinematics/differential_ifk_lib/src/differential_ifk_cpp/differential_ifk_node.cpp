#include <ros/ros.h>
#include "DifferentialIfkNode.hpp"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "differential_ifk");

	DifferentialIfkNode node;
	node.spin();

	return 0;
}
