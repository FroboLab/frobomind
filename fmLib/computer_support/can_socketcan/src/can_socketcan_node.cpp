#include <csignal>
#include <cstdio>
#include "ros/ros.h"

#include "SocketCan.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "can_node");

  ros::NodeHandle n("~");
  ros::NodeHandle nh;

  std::string device,publisher_topic,subscriber_topic;

  n.param<std::string>("device", device, "can0");
  n.param<std::string>("can_from_device_pub", publisher_topic, "/fmSignal/can_from_device");
  n.param<std::string>("can_to_device_sub", subscriber_topic, "/fmSignal/can_to_device");

  SocketCan can;


  if(can.initInterface(device)<0)
  {
	  ROS_ERROR("Could not open device: %s",device.c_str());
	  return -1;
  }
  else
  {
	  can.can_rx_publisher_ = nh.advertise<msgs::can>(publisher_topic.c_str(),20);
	  can.can_tx_subscriber_ = nh.subscribe(subscriber_topic.c_str(),20,&SocketCan::processCanTxEvent,&can);
	  ros::spin();
  }

  return 0;
}

