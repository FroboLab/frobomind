#include "roboteq_sdc2130/sdc2130.hpp"

int main (int argc, char** argv)
{
	ros::init(argc,argv,"roboteq_controller");
	sdc2130 controller;
	controller.spin();

	return 0;
}
