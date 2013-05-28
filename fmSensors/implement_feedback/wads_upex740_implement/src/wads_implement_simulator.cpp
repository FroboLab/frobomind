#include "wads_implement_simulator.hpp"


WADSImplementSimulator::WADSImplementSimulator() : rosNode("~")
{
  // Parameters
  setupParametersAll();

  // Topics
  this->sensorPublisher = this->rosNode.advertise<std_msgs::Float64>(this->publisherTopic, 10);
  this->laserSubscriber = this->rosNode.subscribe(this->subscriberTopic, 10, &WADSImplementSimulator::processLaserScan, this);
}

void WADSImplementSimulator::setupParametersAll()
{
  // Read parameters from ROS
  this->rosNode.param<double>("sensor_width", this->sensor_width_, 2.0);
  this->rosNode.param<double>("sensor_length", this->sensor_length_, 1.0);
  this->rosNode.param<double>("sensor_offset_x", this->sensor_offset_x_, 0.6);
  this->rosNode.param<double>("sensor_offset_y", this->sensor_offset_y_, 0.0);
  this->rosNode.param<double>("sensor_gain", this->sensor_gain_, 0.1);
  this->rosNode.param<double>("sensor_outerrange", this->sensor_outerrange_, 0.1);
  this->rosNode.param<double>("sensor_max", this->sensor_max_, 5.0);
  this->rosNode.param<int>("sensor_resolution", this->sensor_resolution_, 1024);
  this->rosNode.param<std::string>("subscriberTopic", this->subscriberTopic, "/not_configured/laser");
  this->rosNode.param<std::string>("publisherTopic", this->publisherTopic, "/not_configured/sensor");

  // Calulate internal parameters
  setupParametersScanArea();
  setupParametersScanResolution();
}
void WADSImplementSimulator::setupParametersScanArea()
{
  // Calculate sensor corners position in reference coordinates
  scan_center_.x = sensor_offset_x_ + (sensor_length_/2);
  scan_center_.y = sensor_offset_y_;
  scan_x_min_ = sensor_offset_x_ - sensor_outerrange_;
  scan_x_max_ = sensor_offset_x_ + sensor_length_ + sensor_outerrange_;
  scan_x_total_ = sensor_length_ + 2 * sensor_outerrange_;
  scan_y_min_ = sensor_offset_y_ - (sensor_width_/2) - sensor_outerrange_;
  scan_y_max_ = sensor_offset_y_ + (sensor_width_/2) + sensor_outerrange_;
  scan_y_total_ = sensor_width_ + 2 * sensor_outerrange_;

  ROS_INFO("center: %f, %f", scan_center_.x, scan_center_.y);
  ROS_INFO("max: %f, %f", scan_x_max_, scan_y_max_);
  ROS_INFO("min: %f, %f", scan_x_min_, scan_y_min_);

  // Calculate outer angles that is needed to cover the scanner area
  //scan_angle_min_ = std::atan2(y_min, x_min);
  //scan_angle_max_ = std::atan2(y_max, x_min);

  // Calculate max distance needed to cover scanner area
  if (sensor_offset_y_ <= 0)
    {
      scan_distance_max_ = std::sqrt(scan_x_max_ * scan_x_max_ + scan_y_min_ * scan_y_min_);
    }
  else
    {
      scan_distance_max_ = std::sqrt(scan_x_max_ * scan_x_max_ + scan_y_max_ * scan_y_max_);
    }
}
void WADSImplementSimulator::setupParametersScanResolution()
{
  scan_resolution_step_ = sensor_max_ / sensor_resolution_;
}
void WADSImplementSimulator::processLaserScan(const sensor_msgs::LaserScan::ConstPtr& laser_scan)
{
  // Convert to point cloud
  sensor_msgs::PointCloud scan_cloud;
  projector_.projectLaser(*laser_scan, scan_cloud, scan_distance_max_);

  // Make sensor estimate
  sensorEstimation(scan_cloud);

  // Publish reading to topic
  std_msgs::Float64 outvalue;
  outvalue.data = sensor_value_;
  this->sensorPublisher.publish(outvalue);
}
void WADSImplementSimulator::sensorEstimation(const sensor_msgs::PointCloud& scan_cloud)
{
  // Work values
  float distance_center;
  float dx, dy, px, py;
  int found_points = 0;
  geometry_msgs::Point32 mass_center;

  // Reset last value
  sensor_value_ = 0;

  BOOST_FOREACH( geometry_msgs::Point32 pnt, scan_cloud.points )
    {
      // In outer ROI?
      if (
	  (pnt.y < scan_y_max_) &&
	  (pnt.y > scan_y_min_) &&
	  (pnt.x < scan_x_max_) &&
	  (pnt.x > scan_x_min_)
	  )
	{
	  // We found a point
	  found_points++;
	  
	  // Add to mass center
	  mass_center.x += pnt.x;
	  mass_center.y += pnt.y;
	}
    }

  if (0 < found_points)
    {
      // Average
      mass_center.x /= scan_cloud.points.size();
      mass_center.y /= scan_cloud.points.size();

      // Distance to sensor edge
      dx = std::abs(scan_center_.x - mass_center.x);
      dy = std::abs(scan_center_.y - mass_center.y);



      // Relative interted distance from center
      //px = 1 - (2*dx)/scan_x_total_;
      //py = 1 - (2*dy)/scan_y_total_;

      // Manhattan distance
      //distance_center = (px+py)/2;

      // SUM values
      //sensor_value_ += distance_center;

      // Mass function
      //sensor_value_ *= std::sqrt(found_points);

      // Undepend on number of found points
      //sensor_value_ /= found_points;

      // Gain
      // Sensor model: 43.6719 - 4.5 x - 50.625 x^2 - 11.25 y + 3.825 x y - 20.3906 y^2
      sensor_value_ = std::abs(43.6719 - 4.5*dx - 50.625*dx*dx - 11.25*dy + 3.825*dx*dy - 20.3906*dy*dy);
      sensor_value_ *= sensor_gain_;

      // Clipping
      if (sensor_value_ > sensor_max_)
	sensor_value_ = sensor_max_;

      if (sensor_value_ < 0)
	sensor_value_ = 0;

      // Resolution fit
      sensor_value_ -= std::fmod(sensor_value_,scan_resolution_step_);
    }
}

void WADSImplementSimulator::spin()
{
  ros::spin();
}
