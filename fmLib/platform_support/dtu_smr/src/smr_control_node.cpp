/****************************************************************************
 # DTU SMR 
 # Copyright (c) 2013 Morten Kjaergaard
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 #	* Redistributions of source code must retain the above copyright
 #  	notice, this list of conditions and the following disclaimer.
 #	* Redistributions in binary form must reproduce the above copyright
 #  	notice, this list of conditions and the following disclaimer in the
 #  	documentation and/or other materials provided with the distribution.
 #	* Neither the name FroboMind nor the
 #  	names of its contributors may be used to endorse or promote products
 #  	derived from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 # ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 # DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 # DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 # (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 # ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 # SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>

#include "connection.cpp"

#include <boost/tokenizer.hpp>

class SMRControlNode
{

  //Ros objects
  ros::NodeHandle nh_;

  // Transform
  //tf::TransformListener tf_listener_;

  ros::Publisher odom_publisher_;
  ros::Publisher clock_publisher_;
  tf::TransformBroadcaster tf;

  // Subscriptions
  ros::Subscriber smrcl_cmd_subscriber_;
  ros::Subscriber cmd_vel_subscriber_;

  Connection conn;

  double last_x;
  double last_y;
  double last_t;
  bool first;

  int watchdog_;

  nav_msgs::Odometry odom_message;

  const static float PI = 3.14159265;

  void smrclCmdHandler( const std_msgs::String::ConstPtr& cmd )
  {
    conn.sendLine(cmd->data.c_str());
  }

  void cmdVelHandler( const geometry_msgs::Twist::ConstPtr& vel )
  {
    double dist = 0.15;
    double dx = vel->linear.x;
    double dt = -vel->angular.z;
    double m_left = dx + dist * dt;
    double m_right = dx - dist * dt;

    watchdog_ = 5;

    char command[64];
    sprintf(command, "motorcmds %4.2f %4.2f", m_left, m_right );
    std::cout << std::string(command) << std::endl;
    conn.sendLine( std::string(command) );
  }

public:
  SMRControlNode(ros::NodeHandle& nh) :
    nh_(nh),
    odom_publisher_(nh_.advertise<nav_msgs::Odometry> ("/odom", 1)),
    clock_publisher_(nh_.advertise<rosgraph_msgs::Clock>("/clock",10)),
    smrcl_cmd_subscriber_( nh_.subscribe<std_msgs::String>("/smrcl_cmd", 10, &SMRControlNode::smrclCmdHandler, this) ),
    cmd_vel_subscriber_( nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &SMRControlNode::cmdVelHandler, this) ),
    first( true )
  {
    ROS_INFO("Started SMR Control Node");

//    bool result = conn.connect("192.38.66.85",31001);
    bool result = conn.connect("192.168.2.210",31001);
    ROS_INFO("Connect %i", result);

    odom_message.header.frame_id = "/odom";
  }

  ~SMRControlNode()
  {
  }



  void parse( const std::string& data )
  {
    boost::regex expr("stream\\s+(-?[\\d.]+)\\s+(-?[\\d.]+)\\s+(-?[\\d.]+)\\s+(-?[\\d.]+)");
    boost::smatch what;
    if (boost::regex_search(data, what, expr))
    {
      double x = atof(what[2].str().c_str());
      double y = atof(what[3].str().c_str());
      double t = atof(what[4].str().c_str());
      double time = atof(what[1].str().c_str());

      // Create ros::Time
      ros::Time ros_time;
      ros_time.fromSec(time);

      // broadcast odometry transform
      tf::Transform txOdom( tf::createQuaternionFromYaw(t), tf::Point(x, y, 0.0) );
      tf.sendTransform( tf::StampedTransform(txOdom, ros_time, "/odom", "/base_footprint") );

      if( !first ) // Skip odometry first time since we need valid last_* values
      {
        // broadcast odometry message
        nav_msgs::Odometry odom;
        tf::poseTFToMsg( txOdom, odom.pose.pose );
        odom.twist.twist.linear.x = (last_x - x)*0.1;
        odom.twist.twist.linear.y = (last_y - y)*0.1;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = (last_t - t)*0.1;

        odom.header.frame_id = "/odom";
        odom.header.stamp = ros_time;

        odom_publisher_.publish(odom);
      }

      // Save last position
      last_x = x;
      last_y = y;
      last_t = t;
      first = false;
 
      // broadcast clock
      rosgraph_msgs::Clock clockMsg;
      clockMsg.clock = ros_time;
      clock_publisher_.publish(clockMsg);

 //     ROS_INFO("Published");

    }
    else
    {
      ROS_INFO("Parse Error %s", data.c_str());
    }
  }

  void run(int argc, char** argv)
  {
    // Initialize ROS

    std::string data;

    conn.sendLine("stream 10 \"$odox\" \"$odoy\" \"$odoth\"");

    ros::Rate r(10);
    while (nh_.ok())
    {
      std::cout << conn.receiveData() << std::endl;
      while( conn.popLine(data) )
      {
        parse(data);
      }
      ros::spinOnce();

      if( watchdog_ == 0 )
      {
//        conn.sendLine("stop");
      }
      else
      {
        --watchdog_;
      }

      r.sleep();
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "smr_control_node");
  ros::NodeHandle nh;
  SMRControlNode component(nh);

  component.run(argc, argv);
  return 0;
}

