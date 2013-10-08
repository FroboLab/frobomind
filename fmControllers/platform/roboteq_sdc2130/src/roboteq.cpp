#include "roboteq_sdc2130/roboteq.hpp"

RoboTeQ::RoboTeQ()
{
	ff = fs = 0;
	two_channel = true;
}

/* Takes number of arguments, command string and an arbitrary number of integer arguments */
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

/* Callback for handling serial message*/
void RoboTeQ::serialCallback(const msgs::serial::ConstPtr& msg)
{
	ROS_DEBUG("Message received %s",msg->data.c_str());
	last_serial_msg = ros::Time::now();
	status.online = true;

	std::stringstream ss;
	char dummy[25];

	if(sscanf(msg->data.c_str(),"+%s",dummy))
	{
		ss << dummy << " ";
	}
	else if(sscanf(msg->data.c_str(),"CB=%d:%d", &cb1,&cb2))
	{
		hall_feedback(msg->header.stamp, cb1, cb2);
	}
	else if(sscanf(msg->data.c_str(),"CB=%d",&cb1))
	{
		hall_feedback(msg->header.stamp, cb1);
	}
	else if(sscanf(msg->data.c_str(),"FF=%d",&ff))
	{
		ss << faultFlagsToString(ff) << " ";
	}
	else if(sscanf(msg->data.c_str(),"FS=%d",&fs))
	{
		ss << statusFlagsToString(fs) << " ";
	}
	else if(sscanf(msg->data.c_str(),"V=%d:%d:%d",&v1,&v2,&v3))
	{
		ss << "internal_voltage:" << v1/10.0 << "battery_voltage:" << v2/10.0 << " " << "5V_output_voltage:" << v3/1000.0 << " ";
	}
	else if(sscanf(msg->data.c_str(),"T=%d:%d",&t1,&t2))
	{
		ss << "temp_ic:" << t1 << " temp_ch1:" << t2 << " ";
	}
	else if(sscanf(msg->data.c_str(),"T=%d:%d:%d",&t1,&t2,&t3))
	{
		ss << "temp_ic:" << t1 << " temp_ch1:" << t2 << " temp_ch2:" << t3 << " ";
	}
	else if(sscanf(msg->data.c_str(),"A=%d",&a1))
	{
		ss << "motor_amps_ch1:" << a1/10.0 << " ";
	}
	else if(sscanf(msg->data.c_str(),"A=%d:%d",	&a1,&a2))
	{
		ss << "motor_amps_ch1:" << a1/10.0 << " motor_amps_ch1:" << a2/10.0 << " ";
	}
	else if(sscanf(msg->data.c_str(),"BA=%d",&ba1))
	{
		ss << "battery_amps:" << ba1/10.0 << " ";
	}
	else if(sscanf(msg->data.c_str(),"BA=%d:%d",&ba1,&ba2))
	{
		ss << "battery_amps_ch1:" << ba1/10.0 << " " << "battery_amps_ch2:" << ba2/10.0 << " " ;
	}
	else if(sscanf(msg->data.c_str(),"FID=%s",dummy))
	{
		status.online = true;
		status.initialised = false;
		ROS_WARN("Found %s",msg->data.c_str());
		initController("Standard");
	}
	else if(sscanf(msg->data.c_str(),"-%s",dummy))
	{
		ROS_WARN("RoboTeQ did not acknowledge the transmit");
	}
	else
	{
		ROS_WARN("RoboTeQ: %s",msg->data.c_str());
	}
	status_out.header.stamp = msg->header.stamp;
	status_out.data = ss.str();
	status_publisher.publish(status_out);

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
