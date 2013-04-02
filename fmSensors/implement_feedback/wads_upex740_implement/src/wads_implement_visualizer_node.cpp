#include "wads_implement_visualizer.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "WADSVisualizer");

  WADSVisualizer wadsNode;
  wadsNode.spin();

  return 0;
}
