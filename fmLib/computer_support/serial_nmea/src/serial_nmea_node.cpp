/*************************************************************************************
# Copyright (c) 2011-2014, Søren Hundevadt Nielsen, Leon Bonde Larsen, Mathias Mikkel Neerup, Kjeld Jensen
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
# File:     serial_nmea_combined_node.cpp                                            
# Purpose:  Create an interface node to handle NMEA communication over a serial interface.
# Project:  FroboMind
# Author:   Søren Hundevadt Nielsen <soeni05@gmail.com>	
			Leon Bonde Larsen <leon@bondelarsen.dk>
			Mathias Mikkel Neerup <manee12@student.sdu.dk>
			Kjeld Jensen <kjeld@frobomind.org>
# Created:  Apr 29, 2011 Søren Hundevadt Nielsen, Source written
# Modified: 2014-03-07 Mathias & Kjeld, combined NMEA and serial interface
*************************************************************************************/
#include <string>

#include "ros/ros.h"
#include "msgs/serial.h"
#include "msgs/nmea.h"
#include "serial_interface.h"

int main(int argc, char **argv)
{
  /* parameters */
  std::string device;
  std::string topic_nmea_from_device_pub;
  std::string topic_nmea_to_device_sub;
  int baudrate;
  bool use_checksum;

  /* initialize ros usage */
  ros::init(argc, argv, "serial_nmea_combined");
  ros::Publisher s_publisher;
  ros::Subscriber s_subscriber;

  /* private nodehandlers */
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  /* read parameters from ros parameter server if available otherwise use default values */
  n.param<std::string> ("nmea_from_device_pub", topic_nmea_from_device_pub, "/fmData/nmea_from_device");
  n.param<std::string> ("nmea_to_device_sub", topic_nmea_to_device_sub, "/fmData/nmea_to_device");
  n.param<std::string> ("serial_device", device, "/dev/ttyS0");
  n.param<int> ("serial_baudrate", baudrate, 115200);
  n.param<bool>("use_nmea_checksum" , use_checksum , true);

  /* set up publisher functions */
  s_publisher = nh.advertise<msgs::nmea> (topic_nmea_from_device_pub.c_str(), 20,1);

  /* initialize serial device */
  serial_interface serial(s_publisher);
  serial.term_char = 10;
  serial.nmea.set_use_checksum(use_checksum); /* tell nmea to use cheksum(or not) */
  int error = serial.openDevice(device, baudrate);
  if (error == 0)
  {
    /* set up subscriber functions */
    s_subscriber = nh.subscribe<msgs::nmea> (topic_nmea_to_device_sub.c_str(), 20, &serial_interface::writeHandler, &serial);

    ros::spin();
  }

  return 0;
}

