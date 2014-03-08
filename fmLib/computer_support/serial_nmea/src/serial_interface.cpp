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
 # File:     serial_interface.cpp
 # Purpose:  Create a interface node to handle serial communication
 # Project:  FroboMind
 # Author:   Søren Hundevadt Nielsen <soeni05@gmail.com>	
			 Leon Bonde Larsen <leon@bondelarsen.dk>
		   	 Mathias Mikkel Neerup <manee12@student.sdu.dk>
			 Kjeld Jensen <kjeld@frobomind.org>
 # Created:  Apr 29, 2011 Søren Hundevadt Nielsen, Source written
 # Modified: 2014-03-07 Mathias & Kjeld, combined NMEA and serial interface
**************************************************************************************/

#include "serial_interface.h"
#include <string>
using namespace std;

serial_interface::serial_interface(ros::Publisher& rx_publisher) : serial_(io_)
{
	rx_publisher_ = rx_publisher;
}

void serial_interface::readHandler(const boost::system::error_code& error, size_t bytes_transferred)
{
	if ((bytes_transferred > 0) && (error == 0)){

		istream is(&readbuffer);

		string str;

		/*
		 * Read a single line from the stream, the stream should contain atleast
		 * one termchar. If there are more than one line in the streambuf we let
		 * boost asio handle that and call us again.
		 * */
		getline(is,str,term_char);

		if(is.bad() || is.fail()){
			ROS_INFO("Failed to extract line from serial ");
		} else {
			//NMEA encode string
			msgs::nmea nmea_msg = nmea.str_to_msg(str);
			//publish msg
			rx_publisher_.publish(nmea_msg);
		}
	}
	serial_interface::readSome();
}

void serial_interface::readSome()
{
	if (ros::ok())
	{
//		serial_.async_read_some(boost::asio::buffer(&readbuffer_, 1), boost::bind(&serial_interface::readHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
		boost::asio::async_read_until(serial_, readbuffer, term_char,
				boost::bind(&serial_interface::readHandler, this,
						boost::asio::placeholders::error,
						boost::asio::placeholders::bytes_transferred));
	}
}

bool serial_interface::openDevice(std::string device, int baudrate)
{
	device_is_open = false;
	try
	{

		boost::asio::serial_port_base::baud_rate BAUD(baudrate);
		serial_.open(device);
		serial_.set_option(BAUD);

	} catch (boost::system::system_error &e)
	{
		ROS_ERROR("Unable to open device %s failed; %s", e.what(), device.c_str());
		return 1;
	}

	device_is_open = true;
	serial_interface::readSome();

	boost::thread t(boost::bind(&boost::asio::io_service::run, &io_));

	return 0;
}

void serial_interface::writeHandler(const msgs::nmea::ConstPtr& msg)
{
	if (serial_.is_open()){
		//Convert from nmea to serial
		msgs::serial serial_msg = nmea.msg_to_str(msg);
		//Write serial to serial-port
		serial_.write_some(boost::asio::buffer(serial_msg.data.c_str(), serial_msg.data.length()));
	}
}

serial_interface::~serial_interface()
{
	if (device_is_open)
	{
		serial_.cancel();
		serial_.close();
	}
	else
		sleep(2);
}
