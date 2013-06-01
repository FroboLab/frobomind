#include "roboteq/roboteq.hpp"

RoboTeQ::RoboTeQ()
{
	ff = fs = 0;
	two_channel = true;
	hall_cb1_initialised = hall_cb2_initialised = false;
	power_cb1_initialised = power_cb2_initialised = false;
	temperature_cb1_initialised = temperature_cb2_initialised = false;
}

/*!Takes number of arguments, command string and an arbitrary number of integer arguments*/
void RoboTeQ::transmit(int args, std::string cmd , ...)
{
	int number;
	va_list arguments;
	va_start(arguments , cmd);

	std::stringstream ss;

	ss << cmd;
	args--;
	while( args )
	{
		number = va_arg( arguments , int);
		ss << " " << number;
		args--;
	}

	ss << "\r";

	serial_out.data = ss.str();
	serial_out.header.stamp = ros::Time::now();
	serial_publisher.publish(serial_out);

	va_end(arguments);
}
void RoboTeQ::transmit(std::string cmd)
{
	serial_out.data = cmd;
	serial_out.header.stamp = ros::Time::now();
	serial_publisher.publish(serial_out);
}
void RoboTeQ::registerHallCb(int channel, void (*callback)(ros::Time time , int value))
{
	if(channel == 1)
	{
		hall_ch1_callback = callback;
		hall_cb1_initialised = true;
	}
	else if(channel == 2 && two_channel)
	{
		hall_ch2_callback = callback;
		hall_cb2_initialised = true;
	}
	else
		ROS_WARN("%s: Error registering callback",ros::this_node::getName().c_str());
}

void RoboTeQ::registerPowerCb(int channel, void (*callback)(ros::Time time , int value))
{
	if(channel == 1)
	{
		power_ch1_callback = callback;
		power_cb1_initialised = true;
	}
	else if(channel == 2 && two_channel)
	{
		power_ch2_callback = callback;
		power_cb2_initialised = true;
	}
	else
		ROS_WARN("%s: Error registering callback",ros::this_node::getName().c_str());
}


/*!Callback for handling serial message*/
void RoboTeQ::serialCallback(const msgs::serial::ConstPtr& msg)
{
	ROS_DEBUG("Message received %s",msg->data.c_str());
	last_serial_msg = ros::Time::now();
	char dummy[25];

	if(sscanf(msg->data.c_str(),"+%s",dummy)){}

	else if(two_channel && sscanf(msg->data.c_str(),"CB=%d:%d",	&cb1,&cb2))
	{
		if(hall_cb1_initialised && hall_cb2_initialised)
			hall_feedback(msg->header.stamp, cb1, cb2);
	}
	else if(two_channel && sscanf(msg->data.c_str(),"P=%d:%d",		&p1,&p2))
	{
		if(power_cb1_initialised && power_cb2_initialised)
			power_feedback(msg->header.stamp, cb1, cb2);
	}
	else if((!two_channel) && sscanf(msg->data.c_str(),"CB=%d",	&cb1))
	{
		if(hall_cb1_initialised)
			hall_feedback(msg->header.stamp, cb1);
	}
	else if((!two_channel) && sscanf(msg->data.c_str(),"P=%d",		&p1))
	{
		if(power_cb1_initialised)
			power_feedback(msg->header.stamp, cb1);
	}
	else if(sscanf(msg->data.c_str(),"FF=%d",		&ff))
	{
		std::stringstream ss;
		ss << statusFlagsToString(fs) << faultFlagsToString(ff) <<
				", battery:" << v2*10 <<
				", temp_ic:" << t1 << ", temp_ch1:" << t2 << ", temp_ch2:" << t3;

		status_out.header.stamp = msg->header.stamp;
		status_out.data = ss.str();
		status_publisher.publish(status_out);
	}
	else if(sscanf(msg->data.c_str(),"V=%d:%d:%d",	&v1,&v2,&v3))
	{ }
	else if(sscanf(msg->data.c_str(),"T=%d:%d:%d",	&t1,&t2,&t3))
	{ }
	else if(sscanf(msg->data.c_str(),"FS=%d",		&fs))
	{ }
//	else if(sscanf(msg->data.c_str(),"A=%d:%d",		&a1,&a2))		{ }
//	else if(sscanf(msg->data.c_str(),"BA=%d:%d",	&ba1,&ba2))		{ }
//	else if(sscanf(msg->data.c_str(),"BS=%d:%d",	&bs1,&bs2))		{ }
//	else if(sscanf(msg->data.c_str(),"BSR=%d:%d",	&bsr1,&bsr2))	{ }
//	else if(sscanf(msg->data.c_str(),"CBR=%d:%d",	&cbr1,&cbr2))	{ }
//	else if(sscanf(msg->data.c_str(),"E=%d:%d",		&e1,&e2))		{ }
//	else if(sscanf(msg->data.c_str(),"F=%d:%d",		&f1,&f2))		{ }
	else if(sscanf(msg->data.c_str(),"FID=%s", 		dummy))
	{
		online = true;
		ROS_WARN("Found %s",msg->data.c_str());
	}
	else if(sscanf(msg->data.c_str(),"-%s",			dummy))
	{
		ROS_WARN("RoboTeQ did not acknowledge the transmit");
	}
	else
	{
		ROS_WARN("RoboTeQ: %s",msg->data.c_str());
	}
}

/*!Converts status flag to elaborated string*/
std::string RoboTeQ::statusFlagsToString(unsigned int status)
{
	std::stringstream ss;

	if(status & 0x01)
		ss << "serial mode, ";
	if(status & 0x02)
		ss << "pulse mode, ";
	if(status & 0x04)
		ss << "analog mode, ";
	if(status & 0x08)
		ss << "power stage off, ";
	if(status & 0x10)
		ss << "stall detected, ";
	if(status & 0x20)
		ss << "at limit, ";
	if(status & 0x40)
		ss << "analog inputs uncalibrated, ";
	if(status & 0x80)
		ss << "amps uncalibrated ";

	return ss.str();
}

/*!Converts fault flag to elaborated string*/
std::string RoboTeQ::faultFlagsToString(unsigned int status)
{
	std::stringstream ss;

	if(status & 0x01)
		ss << "over heat, ";
	if(status & 0x02)
		ss << "over voltage, ";
	if(status & 0x04)
		ss << "under voltage, ";
	if(status & 0x08)
		ss << "short circuit, ";
	if(status & 0x10)
		ss << "emergency stop, ";
	if(status & 0x20)
		ss << "sepex excitation fault, ";
	if(status & 0x40)
		ss << "mosfet failure, ";
	if(status & 0x80)
		ss << "startup configuration fault ";
	if(status == 0)
		ss << "OK";

	return ss.str();
}
