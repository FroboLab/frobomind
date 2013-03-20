#include "wads_implement_visualizer.hpp"

void WADSVisualizer::setupParameters()
{
  this->rosNode.param<double>("sensor_width", this->sensor_width_, 2.0);
  this->rosNode.param<double>("sensor_length", this->sensor_length_, 1.0);
  this->rosNode.param<double>("sensor_offset_x", this->sensor_offset_x_, 0.6);
  this->rosNode.param<double>("sensor_offset_y", this->sensor_offset_y_, 0.0);
  this->rosNode.param<double>("sensor_max", this->sensor_max_, 5.0);
  this->rosNode.param<std::string>("subscriberTopic", this->subscriberTopic, "/not_configured/sensor");
  this->rosNode.param<std::string>("publisherTopic", this->publisherTopic, "/not_configured/marker");
  this->rosNode.param<std::string>("frame_id", this->frame_id, "base_id");
  this->rosNode.param<std::string>("namespace", this->nspace, "namespace");
}

void WADSVisualizer::setupMarker()
{
  this->marker_.header.frame_id = this->frame_id;
  this->marker_.header.stamp = ros::Time();
  this->marker_.ns = this->nspace;
  this->marker_.id = 0;
  this->marker_.type = visualization_msgs::Marker::CUBE;
  this->marker_.action = visualization_msgs::Marker::ADD;
  this->marker_.pose.position.x = this->sensor_offset_x_ + (this->sensor_length_/2);
  this->marker_.pose.position.y = this->sensor_offset_y_;
  this->marker_.pose.position.z = 0;
  this->marker_.pose.orientation.x = 0.0;
  this->marker_.pose.orientation.y = 0.0;
  this->marker_.pose.orientation.z = 0.0;
  this->marker_.pose.orientation.w = 1.0;
  this->marker_.scale.x = this->sensor_length_;
  this->marker_.scale.y = this->sensor_width_;
  this->marker_.scale.z = 0.02;
  this->marker_.color.a = 0.5;
  this->marker_.color.r = 0.0;
  this->marker_.color.g = 1.0;
  this->marker_.color.b = 0.0;
}
void WADSVisualizer::dataCallback(const std_msgs::Float64::ConstPtr& sensorReading)
{
  double color = sensorReading->data / this->sensor_max_;

  this->marker_.color.r = color;
  this->marker_.color.g = 1.0 - color;

  markerPublisher.publish( this->marker_ );
}
WADSVisualizer::WADSVisualizer() : rosNode("~")
{
  // Parameters
  this->setupParameters();
  this->setupMarker();

  // Topics
  this->markerPublisher = this->rosNode.advertise<visualization_msgs::Marker>(this->publisherTopic, 10);
  this->sensorSubscriber = this->rosNode.subscribe(this->subscriberTopic, 10, &WADSVisualizer::dataCallback, this);


}
void WADSVisualizer::spin()
{
  ros::spin();
}
