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

bool NmeaToTranmerc::parseNmea(const msgs::nmea::ConstPtr& msg)
{
	bool out = false;
	try
	{
		// lexical cast is made to local variables to prevent partially updated messages if cast fails
		int fix = boost::lexical_cast<int>(msg->data.at(NMEA_FIX_QUALITY));
		int sat = boost::lexical_cast<int>(msg->data.at(NMEA_SATTELITES));
		std::string time = boost::lexical_cast<std::string>(msg->data.at(NMEA_TIME));
		double lat = nmeaToDegrees(boost::lexical_cast<double>(msg->data.at(NMEA_LATITUDE)),boost::lexical_cast<std::string>(msg->data.at(NMEA_LATITUDE_DIRECTION)));
		double lon = nmeaToDegrees(boost::lexical_cast<double>(msg->data.at(NMEA_LONGITUDE)),boost::lexical_cast<std::string>(msg->data.at(NMEA_LONGITUDE_DIRECTION)));
		double alt = boost::lexical_cast<double>(msg->data.at(NMEA_ALTITUDE));
		double hdop = boost::lexical_cast<double>(msg->data.at(NMEA_HORIZONTAL_DILUTION_OF_PRECISION));
		double geoid_height = boost::lexical_cast<double>(msg->data.at(NMEA_GEOID_HEIGHT));

		gpgga_tranmerc_msg.header.stamp = ros::Time::now();
		gpgga_tranmerc_msg.header.frame_id = frame_id;
		gpgga_tranmerc_msg.time_recv = msg->header.stamp;

		gpgga_tranmerc_msg.fix = fix;
		gpgga_tranmerc_msg.sat = sat;
		gpgga_tranmerc_msg.time = time;
		gpgga_tranmerc_msg.lat = lat;
		gpgga_tranmerc_msg.lon = lon;
		gpgga_tranmerc_msg.alt = alt;
		gpgga_tranmerc_msg.hdop = hdop;
		gpgga_tranmerc_msg.geoid_height = geoid_height;

		out = true;

	} catch (boost::bad_lexical_cast &)
	{
		ROS_WARN("%s: Lexical cast exception. Message dropped.",ros::this_node::getName().c_str());
	}
	return out;
}

bool NmeaToTranmerc::fixIsValid(void)
{
	return (gpgga_tranmerc_msg.fix >= 1);
}

void NmeaToTranmerc::handleValidMessage(const msgs::nmea::ConstPtr& msg)
{
	if(parseNmea(msg))
	{
		if (fixIsValid())
		{
			double easting, northing;

			Convert_Geodetic_To_Transverse_Mercator (gpgga_tranmerc_msg.lat*deg_to_rad, gpgga_tranmerc_msg.lon*deg_to_rad, &easting, &northing);

			gpgga_tranmerc_msg.easting = easting;
			gpgga_tranmerc_msg.northing = northing;
		}
		else
		{
			gpgga_tranmerc_msg.easting = gpgga_tranmerc_msg.northing = TRANMERC_NOT_VALID;
			ROS_WARN("%s: Invalid fix quality: %d",ros::this_node::getName().c_str(),gpgga_tranmerc_msg.fix);
		}
		gpgga_tranmerc_pub.publish (gpgga_tranmerc_msg);
	}
}

void NmeaToTranmerc::makeItSpin()
{
	local_node_handler.param<double> ("transverse_mercator_a", tm_a, 6378137.0); // Equatorial radius, default is generic for WGS-84 datum
	local_node_handler.param<double> ("transverse_mercator_f", tm_f, 0.0033528106647474805); // Flattening, default is generic for WGS-84 datum  (1/298.257223563)
	local_node_handler.param<double> ("transverse_mercator_false_easting", tm_fe, 500000.0); // False Easting, default is generic for UTM projection
	local_node_handler.param<double> ("transverse_mercator_scale_factor", tm_scale, 0.9996); // Scale Factor, default is generic for UTM projection
	local_node_handler.param<double> ("transverse_mercator_origin_latitude", tm_orglat, 0.0); // Origin Latitude, default is generic for UTM projection
	local_node_handler.param<double> ("transverse_mercator_central_meridian", tm_cmer, 9.0); // Central Meridian, default is UTM32
	local_node_handler.param<double> ("transverse_mercator_false_northing", tm_fn, 0.0); // False northing, default is for UTM northern hemisphere
	local_node_handler.param<std::string> ("frame_id", frame_id, "/base");
	local_node_handler.param<std::string> ("nmea_sub", subscribe_topic_id, "fmInformation/gps_nmea");
	local_node_handler.param<std::string> ("gpgga_tranmerc_pub", publish_topic_id, "fmInformation/gpgga_tranmerc_msg");

	Set_Transverse_Mercator_Parameters(tm_a, tm_f, tm_orglat*deg_to_rad, tm_cmer*deg_to_rad, tm_fe, tm_fn, tm_scale);

	ros::Subscriber sub = local_node_handler.subscribe(subscribe_topic_id, 1, &NmeaToTranmerc::onNmeaMessage, this);
	gpgga_tranmerc_pub = local_node_handler.advertise<msgs::gpgga_tranmerc> (publish_topic_id, 1);

	ros::spin();
}
