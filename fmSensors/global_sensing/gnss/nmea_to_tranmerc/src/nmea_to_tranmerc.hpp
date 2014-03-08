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

#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "msgs/nmea.h"
#include "msgs/gpgga_tranmerc.h"
#include "boost/lexical_cast.hpp"
#include <transverse_mercator/tranmerc.h>

#define TRANMERC_NOT_VALID		-1

#define GPGGA_TIME 0
#define GPGGA_LAT 1
#define GPGGA_LAT_HEMISPHERE 2
#define GPGGA_LON 3
#define GPGGA_LON_HEMISPHERE 4
#define GPGGA_FIX 5
#define GPGGA_SATELLITES 6
#define GPGGA_HDOP 7
#define GPGGA_ALTITUDE 8
#define GPGGA_GEOID_HEIGHT 10

class NmeaToTranmerc
{
public:
	std::string subscribe_topic_id, publish_topic_id, frame_id;
	double tm_a, tm_f, tm_fe, tm_scale, tm_orglat, tm_cmer, tm_fn,deg_to_rad;
	msgs::gpgga_tranmerc gpgga_tranmerc_msg;

	ros::NodeHandle global_node_handler;
	ros::NodeHandle local_node_handler;

	ros::Publisher gpgga_tranmerc_pub;

	NmeaToTranmerc();

	void makeItSpin(void);
	double nmeaToDegrees(double pos, std::string);
	void onNmeaMessage(const msgs::nmea::ConstPtr& msg);
};
