#include <ros/ros.h>
#include "wads_implement_simulator.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "WADSImplementSimulator");

  WADSImplementSimulator wadsNode;
  wadsNode.spin();

  return 0;
}
