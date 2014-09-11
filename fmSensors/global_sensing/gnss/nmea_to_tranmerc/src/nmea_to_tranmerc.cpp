/*****************************************************************************
# FroboMind nmea_to_tranmerc
# Copyright (c) 2013-2014, 
#   Leon Bonde Larsen <leon@bondelarsen.dk>, 
#   Kjeld Jensen <kjeld@frobomind.org>
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#	* Redistributions of source code must retain the above copyright
#	  notice, this list of conditions and the following disclaimer.
#	* Redistributions in binary form must reproduce the above copyright
#	  notice, this list of conditions and the following disclaimer in the
#	  documentation and/or other materials provided with the distribution.
#	* Neither the name FroboMind nor the
#	  names of its contributors may be used to endorse or promote products
#	  derived from this software without specific prior written permission.
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
******************************************************************************
History 

2013-08-xx Leon, original code written
2014-03-08 Kjeld, major profiling, bugfixes and documentation.

*****************************************************************************/

#include "nmea_to_tranmerc.hpp"

NmeaToTranmerc::NmeaToTranmerc():
local_node_handler("~"), global_node_handler()
{
	tm_a = tm_f = tm_fe = tm_scale = tm_orglat = tm_cmer = tm_fn = 0;
	deg_to_rad = M_PI/180.0;
}

double NmeaToTranmerc::nmeaToDegrees(double pos, std::string dir)
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

void NmeaToTranmerc::onNmeaMessage(const msgs::nmea::ConstPtr& msg)
{
	if (msg->valid == true)
	{
		if (!msg->type.compare("GPGGA"))
		{
			bool send_msg = false;
			gpgga_tranmerc_msg.header.stamp = ros::Time::now();
			gpgga_tranmerc_msg.header.frame_id = frame_id;
			gpgga_tranmerc_msg.time_recv = msg->header.stamp;

			try
			{
				int fix = boost::lexical_cast<int>(msg->data.at(GPGGA_FIX));
				if (fix >= 1)
				{
					int sat = boost::lexical_cast<int>(msg->data.at(GPGGA_SATELLITES));
					std::string time = boost::lexical_cast<std::string>(msg->data.at(GPGGA_TIME));
					double lat = nmeaToDegrees(boost::lexical_cast<double>(msg->data.at(GPGGA_LAT)),boost::lexical_cast<std::string>(msg->data.at(GPGGA_LAT_HEMISPHERE)));
					double lon = nmeaToDegrees(boost::lexical_cast<double>(msg->data.at(GPGGA_LON)),boost::lexical_cast<std::string>(msg->data.at(GPGGA_LON_HEMISPHERE)));
					double alt = boost::lexical_cast<double>(msg->data.at(GPGGA_ALTITUDE));
					double hdop = boost::lexical_cast<double>(msg->data.at(GPGGA_HDOP));
					double geoid_height = boost::lexical_cast<double>(msg->data.at(GPGGA_GEOID_HEIGHT));

					gpgga_tranmerc_msg.time = time;
					gpgga_tranmerc_msg.lat = lat;
					gpgga_tranmerc_msg.lon = lon;
					gpgga_tranmerc_msg.fix = fix;
					gpgga_tranmerc_msg.sat = sat;
					gpgga_tranmerc_msg.hdop = hdop;
					gpgga_tranmerc_msg.alt = alt;
					gpgga_tranmerc_msg.geoid_height = geoid_height;

					double easting, northing;
					Convert_Geodetic_To_Transverse_Mercator (gpgga_tranmerc_msg.lat*deg_to_rad, gpgga_tranmerc_msg.lon*deg_to_rad, &easting, &northing);

					gpgga_tranmerc_msg.easting = easting;
					gpgga_tranmerc_msg.northing = northing;
					send_msg = true;
				}
				else
				{
					gpgga_tranmerc_msg.time = "";
					gpgga_tranmerc_msg.lat = 0.0;
					gpgga_tranmerc_msg.lon = 0.0;
					gpgga_tranmerc_msg.fix = 0;
					gpgga_tranmerc_msg.sat = 0;
					gpgga_tranmerc_msg.hdop = 0.0;
					gpgga_tranmerc_msg.alt = 0.0;
					gpgga_tranmerc_msg.geoid_height = 0.0;
					gpgga_tranmerc_msg.easting = 0.0;
					gpgga_tranmerc_msg.northing = 0.0;
					send_msg = true;
				}
			}
			catch (boost::bad_lexical_cast &)
			{
				ROS_WARN("%s: Lexical cast exception. Message dropped.",ros::this_node::getName().c_str());
			}
	
			if (send_msg)
			{	
				gpgga_tranmerc_pub.publish (gpgga_tranmerc_msg);
			}
		}
	}
}

void NmeaToTranmerc::makeItSpin()
{
	local_node_handler.param<double> ("/transverse_mercator_a", tm_a, 6378137.0); // Equatorial radius, default is generic for WGS-84 datum
	local_node_handler.param<double> ("/transverse_mercator_f", tm_f, 0.0033528106647474805); // Flattening, default is generic for WGS-84 datum  (1/298.257223563)
	local_node_handler.param<double> ("/transverse_mercator_false_easting", tm_fe, 500000.0); // False Easting, default is generic for UTM projection
	local_node_handler.param<double> ("/transverse_mercator_scale_factor", tm_scale, 0.9996); // Scale Factor, default is generic for UTM projection
	local_node_handler.param<double> ("/transverse_mercator_origin_latitude", tm_orglat, 0.0); // Origin Latitude, default is generic for UTM projection
	local_node_handler.param<double> ("/transverse_mercator_central_meridian", tm_cmer, 9.0); // Central Meridian, default is UTM32
	local_node_handler.param<double> ("/transverse_mercator_false_northing", tm_fn, 0.0); // False northing, default is for UTM northern hemisphere
	local_node_handler.param<std::string> ("frame_id", frame_id, "/base");
	local_node_handler.param<std::string> ("nmea_sub", subscribe_topic_id, "fmInformation/gps_nmea");
	local_node_handler.param<std::string> ("gpgga_tranmerc_pub", publish_topic_id, "fmInformation/gpgga_tranmerc");

	Set_Transverse_Mercator_Parameters(tm_a, tm_f, tm_orglat*deg_to_rad, tm_cmer*deg_to_rad, tm_fe, tm_fn, tm_scale);

	ros::Subscriber sub = local_node_handler.subscribe(subscribe_topic_id, 1, &NmeaToTranmerc::onNmeaMessage, this);
	gpgga_tranmerc_pub = local_node_handler.advertise<msgs::gpgga_tranmerc> (publish_topic_id, 1);

	ros::spin();
}
