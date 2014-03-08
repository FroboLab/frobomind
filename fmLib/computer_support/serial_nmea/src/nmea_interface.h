#ifndef NMEAINTERFACE_H_
#define NMEALINTERFACE_H_

#include <iostream>
#include <string>

#include "ros/ros.h"

#include <boost/asio.hpp>
#include "boost/tokenizer.hpp"
#include "boost/lexical_cast.hpp"
#include "boost/algorithm/string.hpp"
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/thread.hpp>

#include "msgs/nmea.h"
#include "msgs/serial.h"

class nmea_interface {
	//Private methods
	int atox(const char *s);
	std::string xtoa(unsigned char chk);
	unsigned char get_checksum(std::string str);

	//private vars
	msgs::nmea nmea_msg;
	bool use_checksum;
  public:
	//Public methods
    msgs::nmea str_to_msg(const std::string& msg);
	msgs::serial msg_to_str(const msgs::nmea::ConstPtr& msg);
	void set_use_checksum(bool checksum);
};

#endif /* NMEALINTERFACE_H_ */

