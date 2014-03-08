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
 # 4. Neither the name of the <organization> nor the
 #    names of its contributors may be used to endorse or promote products
 #    derived from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY SØREN HUNDEVADT NIELSEN ''AS IS'' AND ANY
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
 # File:     serial_interface.h
 # Purpose:  Create a interface node to handle serial communication
 # Project:  FroboMind
 # Author:   Søren Hundevadt Nielsen <soeni05@gmail.com>	
			 Leon Bonde Larsen <leon@bondelarsen.dk>
		   	 Mathias Mikkel Neerup <manee12@student.sdu.dk>
			 Kjeld Jensen <kjeld@frobomind.org>
 # Created:  Apr 29, 2011 Søren Hundevadt Nielsen, Source written
 # Modified: 2014-03-07 Mathias & Kjeld, combined NMEA and serial interface
**************************************************************************************/

#ifndef SERIALINTERFACE_H_
#define SERIALINTERFACE_H_

#include <iostream>
#include <string>

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/thread.hpp>

#include "ros/ros.h"
#include "msgs/nmea.h"
#include "msgs/serial.h"
#include "nmea_interface.h"

class serial_interface{
private:

  /* private variables */
  ros::Publisher rx_publisher_;
  boost::asio::io_service io_;
  boost::asio::serial_port serial_;
  boost::asio::streambuf readbuffer;
  char device_is_open;

  /* private methods */
  void readSome();
  void readHandler(const boost::system::error_code& error, size_t bytes_transferred);

public:

  /* public methods */
  serial_interface(ros::Publisher& rx_publisher);
  bool openDevice(std::string device, int baudrate);
  void writeHandler(const msgs::nmea::ConstPtr& msg);
  virtual ~serial_interface();

  char term_char;

  //Create instanse of nmeaInterface
  nmea_interface nmea;
};

#endif /* SERIALINTERFACE_H_ */
