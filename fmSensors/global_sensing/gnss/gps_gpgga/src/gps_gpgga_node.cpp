#include <stdio.h>
#include <string.h>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>
#include "msgs/gpgga.h"
#include "boost/tokenizer.hpp"
#include "boost/lexical_cast.hpp"
#include <boost/algorithm/string.hpp>
#include <msgs/serial.h>
ros::Publisher gpgga_pub;
std::string frame_id;
msgs::gpgga gpgga_msg;

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

double nmea_to_deg(double pos, std::string dir)
{
	pos = pos / 100;
	int dd = floor(pos);
	double mm = (pos - dd) * 100;
	double res = dd + (mm / 60);
	if (dir == "S" || dir == "W")
	{
		res = 0 - res;
	}
	return res;
}

void gps_parser(tokenizer& tokens, ros::Time time_recv)
{
	std::vector<std::string> nmea;
	try
	{
		nmea.assign(tokens.begin(), tokens.end());
		if(nmea.at(0) == "GPGGA")
		{
			ROS_DEBUG("Recevied GPGGA string");
			ROS_DEBUG("Size: %ld",nmea.size());
		}
		if (nmea.at(0) == "GPGGA" && (nmea.size() == 14 || nmea.size() == 15 || nmea.size() == 16))
		{
			// !!! we need to check the checksum of the NMEA string here !!!
			// save current time
			gpgga_msg.header.stamp = ros::Time::now();

			// save data received time
			gpgga_msg.time_recv = time_recv;

			// import satellite fix from the NMEA string
			gpgga_msg.fix = boost::lexical_cast<int>(nmea.at(6));
			if (gpgga_msg.fix >= 1)
			{
				// import data from the NMEA string
				gpgga_msg.time = boost::lexical_cast<std::string>(nmea.at(1));
				gpgga_msg.sat = boost::lexical_cast<int>(nmea.at(7));
				gpgga_msg.hdop = boost::lexical_cast<double>(nmea.at(8));
				gpgga_msg.alt = boost::lexical_cast<double>(nmea.at(9));
				gpgga_msg.geoid_height = boost::lexical_cast<double>(
						nmea.at(11));

				// import lat/lon and convert from hdm.m to hd.d
				gpgga_msg.lat = nmea_to_deg(
						boost::lexical_cast<double>(nmea.at(2)),
						boost::lexical_cast<std::string>(nmea.at(3)));
				gpgga_msg.lon = nmea_to_deg(
						boost::lexical_cast<double>(nmea.at(4)),
						boost::lexical_cast<std::string>(nmea.at(5)));
			}
			else
			{
				// reset data
				gpgga_msg.time = "";
				gpgga_msg.lat = 0;
				gpgga_msg.lon = 0;
				gpgga_msg.sat = 0;
				gpgga_msg.hdop = 0;
				gpgga_msg.alt = 0;
				gpgga_msg.geoid_height = 0;
			}

			// publish the gpgga message
			gpgga_pub.publish(gpgga_msg);
		}
	} catch (boost::bad_lexical_cast &)
	{
		ROS_WARN("gps_parser: bad lexical cast");
	}
}


void gpsCallbackString(const msgs::serial::ConstPtr& msg)
{
	boost::char_separator<char> sep("$*,");
	tokenizer::iterator tok_iter;
	tokenizer tokens(msg->data, sep);

	gps_parser(tokens, msg->header.stamp);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gps_parser");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	std::string subscribe_topic_id;
	std::string publish_topic_id;

	n.param<std::string>("subscribe_topic_id", subscribe_topic_id,
			"fmLib/gps_rx");
	n.param<std::string>("publish_topic_id", publish_topic_id,
			"fmInformation/gpgga_msg");

	ros::Subscriber sub = n.subscribe(subscribe_topic_id, 10, gpsCallbackString);
	gpgga_pub = n.advertise<msgs::gpgga>(publish_topic_id, 1);

	ros::spin();
	return 0;
}

