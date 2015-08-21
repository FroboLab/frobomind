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
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include "pugixml.hpp"

#include "connection.cpp"

class SMRLaserNode
{
  //Ros objects
  ros::NodeHandle nh_;

  ros::Publisher scan_publisher_;
  tf::TransformBroadcaster tf;

  Connection conn;

  sensor_msgs::LaserScan laser_data;

  std::string laser_xml;
  bool laser_xml_started;

  const static float PI = 3.14159265;

  static inline double rad( const double angle )
  {
    return (angle / 180.0) * PI;
  }

public:
  SMRLaserNode(ros::NodeHandle& nh) :
    nh_(nh),
    scan_publisher_(nh_.advertise<sensor_msgs::LaserScan> ("/base_scan", 1)),
    conn( 3500 )
  {
    ROS_INFO("Started SMR Laser Node");

    bool result = conn.connect("192.38.66.85",24919);
    ROS_INFO("Connect %i", result);

    // todo: read all this from server
    laser_data.header.frame_id="/laser";
    //laser_data.angle_min = -120.0 / 180.0 * PI;
    //laser_data.angle_max =  120.0 / 180.0 * PI;
    //laser_data.angle_increment = (240.0 / 180.0 * PI) / 682;
    laser_data.scan_time = 0.1;
    laser_data.time_increment = 0.0;
    laser_data.range_min = 0.0;
    laser_data.range_max = 5.0;
  }

  int intValue(char character)
  {
    if( character <= '9' )
    {
      return( (int)character - (int)'0');
    }
    else
    {
      return( (int)character - (int)'a' + 10);
    }
  }

  void parse( const std::string& line )
  {
    boost::smatch what;

    if( line.find("<scanGet") != std::string::npos )
    {
      laser_xml = line;
      laser_xml_started = true;
    }
    else if ( laser_xml_started )
    {
      laser_xml.append( line );
    }

    if ( line.find("</scanGet>") != std::string::npos && laser_xml_started )
    {
      pugi::xml_document doc;
      pugi::xml_parse_result result = doc.load_buffer( laser_xml.c_str(), laser_xml.size() );
      if( result.status == pugi::status_ok )
      {
        // Read Time
        double time = atof( doc.first_child().attribute("tod").value() );
        ros::Time ros_time;
        ros_time.fromSec(time);
        laser_data.header.stamp = ros_time;

        // Read other laser info....
        laser_data.angle_increment = rad( atof( doc.first_child().attribute("interval").value() ) );
        laser_data.angle_min = rad( atof( doc.first_child().attribute("min").value() ) );
        laser_data.angle_max = rad( atof( doc.first_child().attribute("max").value() ) );
        int count = atoi( doc.first_child().attribute("count").value() );

        // Read Laser Data
        const char* data = doc.first_child().child("bin").child_value();
        if( data != 0 && data[0] != 0 )
        {
          laser_data.ranges.clear();
          for( int i = 0; i < count; i++ )
          {
            int dec = intValue( data[1] )           +
                      intValue( data[0] ) * 16      +
                      intValue( data[3] ) * 16 * 16 +
                      intValue( data[2] ) * 16 * 16 * 16;
            float distance = dec * 0.001;
            laser_data.ranges.push_back(distance);
            data += 4;
          }
        }
        else
        {
          ROS_INFO("scanGet XML parse error, no data?");
        }

        scan_publisher_.publish(laser_data);
        ROS_INFO("Published");

        // Transform
        double x = atof( doc.first_child().child("pos3d").attribute("x").value() );
        double y = atof( doc.first_child().child("pos3d").attribute("y").value() );
        double z = atof( doc.first_child().child("pos3d").attribute("z").value() );
        double rx = atof( doc.first_child().child("rot3d").attribute("x").value() );
        double ry = atof( doc.first_child().child("rot3d").attribute("y").value() );
        double rz = atof( doc.first_child().child("rot3d").attribute("z").value() );

        // broadcast laser transform
        tf::Transform txLaser( tf::createQuaternionFromRPY(rx, ry, rz), tf::Point(x, y, z) );
        tf.sendTransform( tf::StampedTransform(txLaser, ros_time, "/base_footprint", "/laser") );

      }
      else
      {
        ROS_INFO("scanGet XML parse error");
      }

      laser_xml_started = false;
      laser_xml.clear();
    }


  }

  void run(int argc, char** argv)
  {
    // Initialize ROS
    ros::Rate r(10);

    conn.sendLine("scanSet width=180");
    //conn.sendLine("scanPush cmd=\"scanGet\"");

    while (nh_.ok())
    {
      std::cout << conn.receiveData() << std::endl;
      std::string data;
      while( conn.popLine(data) )
      {
        parse( data );
      }
      ros::spinOnce();
      conn.sendLine("scanGet");
      r.sleep();
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "smr_laser_node");
  ros::NodeHandle nh;
  SMRLaserNode component(nh);

  component.run(argc, argv);
  return 0;
}

