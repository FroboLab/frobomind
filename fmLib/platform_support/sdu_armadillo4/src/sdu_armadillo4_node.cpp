#include "roboteq/hbl1650.hpp"

int main (int argc, char** argv)
{
	ros::init(argc,argv,"roboteq_controller");
	hbl1650 controller;
	controller.setChannels(false);
	controller.spin();

	return 0;
}
