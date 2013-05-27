#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <msgs/serial.h>
#include <string>
#include <cctype>

class WADSImplementCommunicator
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle n_;
  
  ros::Publisher pub_;
  ros::Subscriber sub_;

  // Parameters
  double gain_;
  double offset_;
  std::string rx_topic_;
  std::string wads_topic_;

  // Stats
  double read_max_;
  double read_min_;
	
  void onMsg(const msgs::serial::ConstPtr& msg)
  {
    // Find starting position of value
    int startpos = msg->data.length();
    for (std::string::const_iterator it = msg->data.end(); it > msg->data.begin(); it--)
      {
	if (isdigit(*it))
	  {
	    startpos--;
	  }
      }

    std::string str = msg->data.substr(startpos);
    std_msgs::Float64 outvalue;
    int value = atoi(str.c_str());
    
    // Stats
    if ((read_min_ == 0) || (read_min_ > value))
      read_min_ = value;
    if ((read_max_ == 0) || (read_max_ < value))
      read_max_ = value;

    //outvalue.data = offset_ + value * gain_;
    outvalue.data = value;
    this->pub_.publish(outvalue);
  }
  void showStats(const ros::TimerEvent&)
  {
    ROS_INFO("was here...");
    ROS_INFO("WADS Values - min: %f \t max: %f", read_min_, read_max_);
  }
  void setupParametersAll()
  {
    read_max_ = 0;
    read_min_ = 0;
    nh_.param<double>("gain", this->gain_, 1);
    nh_.param<double>("offset", this->offset_, 0.0);
    nh_.param<std::string>("rx_topic", this->rx_topic_, "/fmData/wads_rx");
    nh_.param<std::string>("wads_topic", this->wads_topic_, "/wads");
    ROS_INFO("Gain: %f", gain_);
    ROS_INFO("Offset: %f", offset_);
  }

public:
  WADSImplementCommunicator() : n_("~")
  {
    setupParametersAll();
    pub_ = nh_.advertise<std_msgs::Float64>(wads_topic_, 10);
    sub_ = nh_.subscribe(rx_topic_, 10, &WADSImplementCommunicator::onMsg, this);

    ros::Timer timer = nh_.createTimer(ros::Duration(1), &WADSImplementCommunicator::showStats, this);
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



