#ifndef WADSIMPLEMENTSIMULATOR
#define WADSIMPLEMENTSIMULATOR

#include <string>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Float64.h>
#include <boost/foreach.hpp>

class WADSImplementSimulator
{
private:
  void setupParametersAll(void);
  void setupParametersScanArea(void);
  void setupParametersScanResolution(void);
  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& laser_scan);
  void sensorEstimation(const sensor_msgs::PointCloud& scan_cloud);

  laser_geometry::LaserProjection projector_;
  //double32 scan_angle_min_;
  //double32 scan_angle_max_;
  double scan_distance_max_;
  double scan_x_min_;
  double scan_x_max_;
  double scan_y_min_;
  double scan_y_max_;
  double scan_y_total_; // Width + 2*outerrange
  double scan_x_total_; // Length + 2*outerrange
  double scan_resolution_step_;
  geometry_msgs::Point32 scan_center_;
  double sensor_value_;

  double sensor_width_;
  double sensor_length_;
  double sensor_offset_x_;
  double sensor_offset_y_;
  double sensor_gain_;
  double sensor_outerrange_;
  double sensor_max_;
  int sensor_resolution_;

  // ROS stuff
  ros::NodeHandle rosNode;
  ros::Subscriber laserSubscriber;
  ros::Publisher sensorPublisher;
  std::string subscriberTopic;
  std::string publisherTopic;

public:
  WADSImplementSimulator();
  void spin(void);
  //void setSensorGeometry(const double& width, const double& length, const double& offset_x, const double& offset_y, const double& outerrange);
  //void setSensorGain(const double& gain, const double& max, const int& resolution);
  //double getSensorValue();
};

#endif
