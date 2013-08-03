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
 # 4. Neither the name of the <organization> nor the
 #    names of its contributors may be used to endorse or promote products
 #    derived from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY SØREN HUNDEVADT NIESLSEN ''AS IS'' AND ANY
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
 # File:     serialInterface.cpp
 # Purpose:  Create a interface node to handle serial communication
 # Project:  vic_interfaces
 # Author:   Søren Hundevadt Nielsen <soeni05@gmail.com>
 # Created:  Apr 29, 2011 Søren Hundevadt Nielsen, Source written
 *************************************************************************************/

#include "serialInterface.h"
#include <string>
using namespace std;


serialInterface::serialInterface(ros::Publisher& rx_publisher) :
		serial_(io_)
{

	s_rx_publisher_ = rx_publisher;
}

void serialInterface::readHandler(const boost::system::error_code& error,
		size_t bytes_transferred)
{

	if ((bytes_transferred > 0) && (error == 0))
	{

		istream is(&readbuffer);

		string str;

		/*
		 * Read a single line from the stream, the stream should contain atleast
		 * one termchar. If there are more than one line in the streambuf we let
		 * boost asio handle that and call us again.
		 * */
		getline(is,str,term_char);

		if(is.bad() || is.fail())
		{
			ROS_ERROR("%s: Failed to extract line from serial", ros::this_node::getName().c_str());
		}
		else
		{
			/* publish data to ros */
			++serial_rx_msg.header.seq;
			serial_rx_msg.data = str.c_str();
			serial_rx_msg.header.stamp = ros::Time::now();
			s_rx_publisher_.publish(serial_rx_msg);
		}
	}
	serialInterface::readSome();
}

void serialInterface::readSome()
{
	if (serial_.is_open() /*ros::ok()*/)
	{
		try
		{
		//serial_.async_read_some(boost::asio::buffer(&rx_buffer_, 1), boost::bind(&serialInterface::readHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
		boost::asio::async_read_until(serial_, readbuffer, term_char,
				boost::bind(&serialInterface::readHandler, this,
						boost::asio::placeholders::error,
						boost::asio::placeholders::bytes_transferred));
		}
		catch (boost::system::system_error &e)
		{
			ROS_ERROR("%s: Read operation failed. Attempting recovery...",ros::this_node::getName().c_str());
			try{
			recoverConnection();
			} catch (...){ROS_ERROR("%s: Recovery failed!", ros::this_node::getName().c_str());}
		}
		catch (...)
		{
			ROS_ERROR("%s: Write operation failed",ros::this_node::getName().c_str());
		}

	}
}

bool serialInterface::openDevice(std::string device, int baudrate)
{
	ROS_DEBUG("Open Device");
	dev = device;
	baud = baudrate;
	try
	{
		boost::asio::serial_port_base::baud_rate BAUD(baudrate);
		serial_.open(device);
		serial_.set_option(BAUD);
	} catch (boost::system::system_error &e)
	{
		ROS_ERROR("%s: Connection to device %s failed; %s",ros::this_node::getName().c_str(), e.what(), device.c_str());
		if (serial_.is_open())
		{
			try
			{
				ROS_ERROR("%s: Attempting to cancel connection...",ros::this_node::getName().c_str());
				serial_.cancel();
			} catch (...){ROS_ERROR("Attempt to cancel connection failed...");}
			sleep(1);
			try
			{
				ROS_ERROR("%s: Attempting to close connection...",ros::this_node::getName().c_str());
				serial_.close();
			} catch (...) {ROS_ERROR("%s: Attempt to close connection failed...",ros::this_node::getName().c_str());}
			sleep(1);
		}
		return 1;
	}
	/* start the read from the serial device */
	serialInterface::readSome();

	boost::thread t(boost::bind(&boost::asio::io_service::run, &io_));

	return 0;
}

void serialInterface::writeHandler(const msgs::serial::ConstPtr& msg)
{
	if (serial_.is_open())
	{
		try
		{
			serial_.write_some(
					boost::asio::buffer(msg->data.c_str(), msg->data.length()));
		}
		catch (boost::system::system_error &e)
		{
			ROS_ERROR("%s: Write operation failed. Attempting recovery...",ros::this_node::getName().c_str());
			try{
			recoverConnection();
			} catch (...){ROS_ERROR("%s: Recovery failed!",ros::this_node::getName().c_str());}
		}
		catch (...)
		{
			ROS_ERROR("%s: Write operation failed",ros::this_node::getName().c_str());
		}
	}
}

void serialInterface::recoverConnection(void)
{
	//ROS_ERROR("Write operation failed: %s", e.what());
	//handle the 'write_some: Input/output error' caused by interrupted connection
	try
	{
		ROS_ERROR("%s: Attempting to re-open connection...",ros::this_node::getName().c_str());
		ros::Rate r(2);
		while(openDevice(dev,baud))
			r.sleep();

	} catch (...){ROS_ERROR("%s: Attempt to re-open failed.",ros::this_node::getName().c_str());}
}

serialInterface::~serialInterface()
{
	if(serial_.is_open())
	{
		serial_.cancel();
		serial_.close();
	}
}
