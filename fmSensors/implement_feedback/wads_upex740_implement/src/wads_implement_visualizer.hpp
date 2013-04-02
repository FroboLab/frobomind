#ifndef WADSVISUALIZER
#define WADSVISUALIZER

#include <string>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>

class WADSVisualizer
{
 private:
  void dataCallback(const std_msgs::Float64::ConstPtr&);
  void setupParameters(void);
  void setupMarker(void);
  double sensor_width_;
  double sensor_length_;
  double sensor_max_;
  double sensor_offset_x_;
  double sensor_offset_y_;
  visualization_msgs::Marker marker_;

  // ROS stuff
  ros::NodeHandle rosNode;
  ros::Subscriber sensorSubscriber;
  ros::Publisher markerPublisher;
  std::string subscriberTopic;
  std::string publisherTopic;
  std::string frame_id;
  std::string nspace;


 public:
  WADSVisualizer();
  void spin(void);

};

#endif
