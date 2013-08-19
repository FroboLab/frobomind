#include "nmea_to_tranmerc.hpp"

NmeaToTranmerc::NmeaToTranmerc():
local_node_handler("~"), global_node_handler()
{
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

void NmeaToTranmerc::parseNmea(const msgs::nmea::ConstPtr& msg)
{
	// save current time
	gpgga_tranmerc_msg.header.stamp = ros::Time::now();
	gpgga_tranmerc_msg.header.frame_id = frame_id;

	try
	{
		// import data from the gpgga topic
		gpgga_tranmerc_msg.time_recv = msg->header.stamp;
		gpgga_tranmerc_msg.fix = boost::lexical_cast<int>(msg->data.at(NMEA_FIX_QUALITY));;
		gpgga_tranmerc_msg.sat = boost::lexical_cast<int>(msg->data.at(NMEA_SATTELITES));
		gpgga_tranmerc_msg.time = boost::lexical_cast<std::string>(msg->data.at(NMEA_TIME));
		gpgga_tranmerc_msg.lat = nmeaToDegrees(boost::lexical_cast<double>(msg->data.at(NMEA_LATITUDE)),boost::lexical_cast<std::string>(msg->data.at(NMEA_LATITUDE_DIRECTION)));
		gpgga_tranmerc_msg.lon = nmeaToDegrees(boost::lexical_cast<double>(msg->data.at(NMEA_LONGITUDE)),boost::lexical_cast<std::string>(msg->data.at(NMEA_LONGITUDE_DIRECTION)));
		gpgga_tranmerc_msg.alt = boost::lexical_cast<double>(msg->data.at(NMEA_ALTITUDE));
		gpgga_tranmerc_msg.hdop = boost::lexical_cast<double>(msg->data.at(NMEA_HORIZONTAL_DILUTION_OF_PRECISION));
		gpgga_tranmerc_msg.geoid_height = boost::lexical_cast<double>(msg->data.at(NMEA_GEOID_HEIGHT));
	} catch (boost::bad_lexical_cast &)
	{
		ROS_WARN("%s: Lexical cast exception.",ros::this_node::getName().c_str());
	}
}

bool NmeaToTranmerc::fixIsValid(void)
{
	return (gpgga_tranmerc_msg.fix >= 1);
}

void NmeaToTranmerc::handleValidMessage(const msgs::nmea::ConstPtr& msg)
{
	parseNmea(msg);

	if (fixIsValid()) // if the GPS currently has a valid satellite fix
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
		ROS_WARN("%s: Invalid fix quality: %d",ros::this_node::getName().c_str(),gpgga_tranmerc_msg.fix);
	}
	gpgga_tranmerc_pub.publish (gpgga_tranmerc_msg); // publish the message
}

void NmeaToTranmerc::makeItSpin()
{
	// get parameters
	local_node_handler.param<double> ("transverse_mercator_a", tm_a, 6378137.0); // Equatorial radius, default is generic for WGS-84 datum
	local_node_handler.param<double> ("transverse_mercator_f", tm_f, 0.0033528106647474805); // Flattening, default is generic for WGS-84 datum  (1/298.257223563)
	local_node_handler.param<double> ("transverse_mercator_false_easting", tm_fe, 500000.0); // False Easting, default is generic for UTM projection
	local_node_handler.param<double> ("transverse_mercator_scale_factor", tm_scale, 0.9996); // Scale Factor, default is generic for UTM projection
	local_node_handler.param<double> ("transverse_mercator_origin_latitude", tm_orglat, 0.0); // Origin Latitude, default is generic for UTM projection
	local_node_handler.param<double> ("transverse_mercator_central_meridian", tm_cmer, 9.0); // Central Meridian, default is UTM32
	local_node_handler.param<double> ("transverse_mercator_false_northing", tm_fn, 0.0); // False northing, default is for UTM northern hemisphere
	local_node_handler.param<std::string> ("frame_id", frame_id, "/base");

	// configure transverse mercator parameters
	Set_Transverse_Mercator_Parameters(tm_a, tm_f, tm_orglat*deg_to_rad, tm_cmer*deg_to_rad, tm_fe, tm_fn, tm_scale);

	// get topic names
	local_node_handler.param<std::string> ("nmea_sub", subscribe_topic_id, "fmInformation/gps_nmea");
	local_node_handler.param<std::string> ("gpgga_tranmerc_pub", publish_topic_id, "fmInformation/gpgga_tranmerc_msg");

	// setup subscription topic callbacks
	ros::Subscriber sub = local_node_handler.subscribe(subscribe_topic_id, 1, &NmeaToTranmerc::onNmeaMessage, this);
	gpgga_tranmerc_pub = local_node_handler.advertise<msgs::gpgga_tranmerc> (publish_topic_id, 1);

	ros::spin();
}
