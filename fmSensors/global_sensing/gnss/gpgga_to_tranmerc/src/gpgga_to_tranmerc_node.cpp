/****************************************************************************
# FroboMind gpgga_to_tranmerc
# Copyright (c) 2011-2014, Kjeld Jensen <kjeld@frobolab.org>
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
*****************************************************************************
#
# 2014-09-11 Kjeld, changed transverse mercator launch parameters to global
****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "msgs/gpgga.h"
#include "msgs/gpgga_tranmerc.h"
#include "boost/tokenizer.hpp"
#include "boost/lexical_cast.hpp"
#include <boost/algorithm/string.hpp>
#include <transverse_mercator/tranmerc.h>

#define TRANMERC_NOT_VALID		-1

ros::Publisher gpgga_tranmerc_pub;
std::string frame_id;
msgs::gpgga_tranmerc gpgga_tranmerc_msg;
double deg_to_rad;

void gpsStateCallback(const msgs::gpgga::ConstPtr& msg)
{
	// save current time
	gpgga_tranmerc_msg.header.stamp = ros::Time::now();
	gpgga_tranmerc_msg.header.frame_id = frame_id;

	// import data from the gpgga topic
	gpgga_tranmerc_msg.time_recv = msg->time_recv;
	gpgga_tranmerc_msg.fix = msg->fix;
	gpgga_tranmerc_msg.sat = msg->sat;
	gpgga_tranmerc_msg.time = msg->time;
	gpgga_tranmerc_msg.lat = msg->lat;
	gpgga_tranmerc_msg.lon = msg->lon;
	gpgga_tranmerc_msg.alt = msg->alt;
	gpgga_tranmerc_msg.hdop = msg->hdop;			
	gpgga_tranmerc_msg.geoid_height = msg->geoid_height;

	if (msg->fix >= 1) // if the GPS currently has a valid satellite fix
	{
		double easting, northing;

		// convert lat/lon to transverse mercator projection coordinates
		Convert_Geodetic_To_Transverse_Mercator (gpgga_tranmerc_msg.lat*deg_to_rad, gpgga_tranmerc_msg.lon*deg_to_rad, &easting, &northing);

		gpgga_tranmerc_msg.easting = easting;
		gpgga_tranmerc_msg.northing = northing;
	}
	else
	{
		gpgga_tranmerc_msg.easting = TRANMERC_NOT_VALID;
		gpgga_tranmerc_msg.northing = TRANMERC_NOT_VALID;
	}
	gpgga_tranmerc_pub.publish (gpgga_tranmerc_msg); // publish the message
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gpgga_to_tranmerc");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	std::string subscribe_topic_id;
	std::string publish_topic_id;

	double tm_a, tm_f, tm_fe, tm_scale, tm_orglat, tm_cmer, tm_fn; 

	deg_to_rad = M_PI/180.0;

	// get parameters
	n.param<double> ("/transverse_mercator_a", tm_a, 6378137.0); // Equatorial radius, default is generic for WGS-84 datum
	n.param<double> ("/transverse_mercator_f", tm_f, 0.0033528106647474805); // Flattening, default is generic for WGS-84 datum  (1/298.257223563)
	n.param<double> ("/transverse_mercator_false_easting", tm_fe, 500000.0); // False Easting, default is generic for UTM projection
	n.param<double> ("/transverse_mercator_scale_factor", tm_scale, 0.9996); // Scale Factor, default is generic for UTM projection
	n.param<double> ("/transverse_mercator_origin_latitude", tm_orglat, 0.0); // Origin Latitude, default is generic for UTM projection
	n.param<double> ("/transverse_mercator_central_meridian", tm_cmer, 9.0); // Central Meridian, default is UTM32
	n.param<double> ("/transverse_mercator_false_northing", tm_fn, 0.0); // False northing, default is for UTM northern hemisphere
	n.param<std::string> ("frame_id", frame_id, "/base");

	// configure transverse mercator parameters
	Set_Transverse_Mercator_Parameters(tm_a, tm_f, tm_orglat*deg_to_rad, tm_cmer*deg_to_rad, tm_fe, tm_fn, tm_scale);

	// get topic names
	n.param<std::string> ("gpgga_sub", subscribe_topic_id, "fmInformation/gpgga_msg");
	n.param<std::string> ("gpgga_tranmerc_pub", publish_topic_id, "fmInformation/gpgga_tranmerc_msg");

	// setup subscription topic callbacks
	ros::Subscriber sub = n.subscribe(subscribe_topic_id, 1, gpsStateCallback);
	gpgga_tranmerc_pub = n.advertise<msgs::gpgga_tranmerc> (publish_topic_id, 1);

	ros::spin();
	return 0;
}

