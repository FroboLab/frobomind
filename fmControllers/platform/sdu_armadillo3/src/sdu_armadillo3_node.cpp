#include "roboteq_hbl1650/hbl1650.hpp"

int main (int argc, char** argv)
{
	ros::init(argc,argv,"roboteq_controller");
	hbl1650 controller;
	controller.spin();

	return 0;
}
