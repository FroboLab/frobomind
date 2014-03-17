/*************************************************************************************
# Copyright (c) 2011, Søren Hundevadt Nielsen
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. All advertising materials mentioning features or use of this software
#    must display the following acknowledgement:
#    This product includes software developed by the University of Southern Denmark.
# 4. Neither the name of the University of Southern Denmark nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY SØREN HUNDEVADT NIELSEN "AS IS" AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL SØREN HUNDEVADT NIELSEN BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**************************************************************************************
# File:     serial_node.cpp                                            
# Purpose:  Create a interface node to handle serial communication
# Project:  vic_interfaces
# Author:   Søren Hundevadt Nielsen <soeni05@gmail.com>
# Created:  Apr 29, 2011 Søren Hundevadt Nielsen, Source written
*************************************************************************************/
#include <string>

#include "ros/ros.h"
#include "msgs/serial.h"
#include "math.h"

#include <iostream>
#include <stdint.h>

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <msgs/IntStamped.h>

#define RAD_2_DEG(x) (x/M_PI*180)


int main(int argc, char **argv)
{
  /* ros messages */

  /* parameters */
  std::string device;
  int baudrate=38400;
  int encoder_offset;

  /* initialize ros usage */
  ros::init(argc, argv, "vic_serial_interface");

  tf::TransformBroadcaster link_broadcaster;
  geometry_msgs::TransformStamped link_trans;
   /* private nodehandlers */
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  ros::Publisher enc_pub;
  msgs::IntStamped enc;
  enc_pub = n.advertise<msgs::IntStamped>("/fmInformation/encoder_value", 25); 

  /* read parameters from ros parameter server if available otherwise use default values */
  n.param<std::string> ("device", device, "/dev/ttyUSB0");
  n.param<int>("encoder_offset", encoder_offset, 0);

  /* keep trying until connection is made */
  ros::Rate loop(300);
  uint8_t wr_cmd = 0x0;
  uint16_t in_cmd = 0x0;

  boost::asio::io_service io;
  boost::asio::serial_port serial_(io,device);
  serial_.set_option(boost::asio::serial_port_base::baud_rate(38400));


  while(ros::ok())
  {
	 boost::asio::write(serial_,boost::asio::buffer(&wr_cmd,1));
	 loop.sleep();
	 boost::asio::read(serial_,boost::asio::buffer(&in_cmd,2));
	 uint16_t val = ((in_cmd & 0xff)) << 8 | (in_cmd >> 8);
	 int16_t vl = val - encoder_offset;
	 if( vl < 0)
	 {
		 vl = 16384 + vl;
	 }

	 double angle = vl/16384.0*2*M_PI;
	 double valTwo = 0.58;
	 double roll = -atan(valTwo*sin(angle));
	 double pitch = atan(valTwo*cos(angle));
	 //std::cout << "val: " <<  angle << std::endl;
	 //std::cout << "roll: " << RAD_2_DEG(roll) << std::endl;
	 //std::cout << "pitch: " << RAD_2_DEG(pitch) << std::endl;

	 link_trans.header.stamp = ros::Time::now();
	 link_trans.header.frame_id = "base_link";
	 link_trans.child_frame_id = "laser";
	 link_trans.transform.translation.x = 0;
	 link_trans.transform.translation.y = 0;
	 link_trans.transform.translation.z = 0;
	 link_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,0);
	 link_broadcaster.sendTransform(link_trans);


	enc.header.stamp = ros::Time::now();
	enc.header.frame_id = "laser";
	enc.data = vl;
	enc_pub.publish(enc);




	 ros::spinOnce();
  }

  std::cout << "done" << std::endl;

  return 0;

}

