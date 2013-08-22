#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "msgs/nmea.h"
#include "msgs/gpgga_tranmerc.h"
#include "boost/lexical_cast.hpp"
#include <transverse_mercator/tranmerc.h>

#define TRANMERC_NOT_VALID		-1

#define NMEA_TIME 0
#define NMEA_LATITUDE 1
#define NMEA_LATITUDE_DIRECTION 2
#define NMEA_LONGITUDE 3
#define NMEA_LONGITUDE_DIRECTION 4
#define NMEA_FIX_QUALITY 5
#define NMEA_SATTELITES 6
#define NMEA_HORIZONTAL_DILUTION_OF_PRECISION 7
#define NMEA_ALTITUDE 8
#define NMEA_GEOID_HEIGHT 9
#define NMEA_LAST_DGPS 10
#define NMEA_DGPS_ID 11

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
	bool parseNmea(const msgs::nmea::ConstPtr&);
	bool fixIsValid(void);
	void handleValidMessage(const msgs::nmea::ConstPtr&);
	void onNmeaMessage(const msgs::nmea::ConstPtr& msg) {

	}
};
