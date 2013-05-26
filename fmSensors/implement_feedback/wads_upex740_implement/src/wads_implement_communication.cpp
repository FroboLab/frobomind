#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <msgs/serial.h>
#include <string>

class WADSImplementCommunicator
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle n_;
  
  ros::Publisher pub_;
  ros::Subscriber sub_;
	
  void onMsg(const msgs::serial::ConstPtr& msg)
  {
    std::string str = msg->data.substr(6);
    std_msgs::Float64 outvalue;
    int value = atoi(str.c_str());
    ROS_INFO("String: %s Int: %i", str.c_str(), value);
    outvalue.data = 0;
    this->pub_.publish(outvalue);
  }

public:
  WADSImplementCommunicator() : n_("~")
  {
    pub_ = nh_.advertise<std_msgs::Float64>("/wads", 10);
    sub_ = nh_.subscribe("/fmData/wads_rx", 10, &WADSImplementCommunicator::onMsg, this);
  }
  void spin()
  {
    ros::spin();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wads_implement_communicator");
  WADSImplementCommunicator n;
  n.spin();
}



