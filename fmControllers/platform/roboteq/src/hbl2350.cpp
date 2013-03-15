#include "roboteq/hbl2350.hpp"

int main (int argc, char** argv)
{
	ros::init(argc,argv,"roboteq_controller");
	hbl2350 controller;
	controller.spin();

	return 0;
}

hbl2350::hbl2350( )
:local_node_handler("~"),global_node_handler()
{
	// Variables for parsing parameters
	std::string 	cmd_vel_ch1_topic,
					cmd_vel_ch2_topic,
					serial_tx_topic,
					serial_rx_topic,
					command_relay_topic,
					deadman_topic,
					encoder_ch1_topic,
					encoder_ch2_topic,
					power_ch1_topic,
					power_ch2_topic,
					status_topic,
					temperature_topic;

	double 			max_time_diff_input;

	// Initialise states and object variables
	deadman_active = false;
	last_deadman_received = ros::Time::now();
	initialised = false;
	online = false;
	last_serial_msg = ros::Time::now();
	cmd_vel_active = false;
	idle = true;
	velocity_ch1 = velocity_ch2 = 0;

	// Parameters not yet on parameter server
	max_rpm = 2000;
	anti_windup_percent = 50;
	max_acceleration = 10000; //0.1*rpm per second
	max_deceleration = 20000; //0.1*rpm per second
	velocity_max = 700; // +/- maximum value sent to controller max 1000

	// Parse from parameter server
	local_node_handler.param<std::string>("cmd_vel_ch1_topic",	cmd_vel_ch1_topic,		"/fmActuators/cmd_vel_ch1");
	local_node_handler.param<std::string>("cmd_vel_ch2_topic",	cmd_vel_ch2_topic,		"/fmActuators/cmd_vel_ch2");
	local_node_handler.param<std::string>("serial_rx_topic",	serial_rx_topic,		"/fmCSP/S0_rx");
	local_node_handler.param<std::string>("serial_tx_topic",	serial_tx_topic,		"/fmCSP/S0_tx");
	local_node_handler.param<std::string>("command_relay_topic",command_relay_topic,	"/fmData/command");
	local_node_handler.param<std::string>("deadman_topic",		deadman_topic,			"/fmHMI/joy");
	local_node_handler.param<std::string>("encoder_ch1_topic",	encoder_ch1_topic,		"/fmSensors/encoder_ch1");
	local_node_handler.param<std::string>("encoder_ch2_topic",	encoder_ch2_topic,		"/fmSensors/encoder_ch2");
	local_node_handler.param<std::string>("power_ch1_topic",	power_ch1_topic,		"/fmSensors/power_ch1");
	local_node_handler.param<std::string>("power_ch2_topic",	power_ch2_topic,		"/fmSensors/power_ch2");
	local_node_handler.param<std::string>("status_topic",		status_topic,			"/fmActuators/status");
//	local_node_handler.param<std::string>("temperature_topic",	temperature_topic,		"/fmActuators/temperature");

	local_node_handler.param<int>("p_gain",	p_gain_ch1,	1);
	local_node_handler.param<int>("i_gain",	i_gain_ch1,	0);
	local_node_handler.param<int>("d_gain",	d_gain_ch1,	0);
	local_node_handler.param<bool>("closed_loop_operation",	closed_loop_operation,	false);
	p_gain_ch2 = p_gain_ch1;
	i_gain_ch2 = i_gain_ch1;
	d_gain_ch2 = d_gain_ch1;

	local_node_handler.param<double>("max_time_diff",max_time_diff_input,0.5);
	max_time_diff = ros::Duration(max_time_diff_input);
	local_node_handler.param<double>("mps_to_rpm",mps_to_rpm,5);

	// Set publisher topics
	setSerialPub( 		local_node_handler.advertise<msgs::serial>(		serial_tx_topic,	10) );
	setEncoderCh1Pub( 	local_node_handler.advertise<msgs::IntStamped>(	encoder_ch1_topic,	10) );
	setEncoderCh2Pub( 	local_node_handler.advertise<msgs::IntStamped>(	encoder_ch2_topic,	10) );
	setPowerCh1Pub( 	local_node_handler.advertise<msgs::IntStamped>(	power_ch1_topic,	10) );
	setPowerCh2Pub( 	local_node_handler.advertise<msgs::IntStamped>(	power_ch2_topic,	10) );
	setStatusPub(		local_node_handler.advertise<msgs::StringStamped>(status_topic,		10) );
//	setTemperaturePub(	local_node_handler.advertise<fmMsgs::StringStamped>(temperature_topic,	10) );

	// Set subscriber topics
	serial_sub = local_node_handler.subscribe<msgs::serial>(serial_rx_topic,10,&hbl2350::onSerial,this);
	command_relay_sub = local_node_handler.subscribe<msgs::serial>(command_relay_topic,10,&hbl2350::onCommand,this);
	cmd_vel_ch1_sub = local_node_handler.subscribe<geometry_msgs::TwistStamped>(cmd_vel_ch1_topic,10,&hbl2350::onCmdVelCh1,this);
	cmd_vel_ch2_sub = local_node_handler.subscribe<geometry_msgs::TwistStamped>(cmd_vel_ch2_topic,10,&hbl2350::onCmdVelCh2,this);
	deadman_sub = local_node_handler.subscribe<std_msgs::Bool>(deadman_topic,10,&hbl2350::onDeadman,this);

}

void hbl2350::spin(void)
{
	// Wait for RoboTeQ to come online
	ros::Rate r(5);
	while(!this->subscribers())
	{
		ROS_INFO_THROTTLE(1,"Waiting for serial node to subscribe");
		r.sleep();
	}
	r.sleep();

	// Initialize timer
	ros::Timer t = global_node_handler.createTimer(ros::Duration(0.1),&hbl2350::onTimer,this);

	ros::spin();
}

void hbl2350::initController(void)
{
	ROS_INFO("Initializing...");
	sleep(1);
	transmit(2,	"^ECHOF", 1 ); sleep(TIME_BETWEEN_COMMANDS);						// Echo is disabled

	transmit(1,	"# C"); sleep(TIME_BETWEEN_COMMANDS);								// Clear buffer
	transmit(1,	"?P"); sleep(TIME_BETWEEN_COMMANDS);								// Request power readings
	transmit(1,	"?V"); sleep(TIME_BETWEEN_COMMANDS);								// Request voltage readings
	transmit(1,	"?T"); sleep(TIME_BETWEEN_COMMANDS);								// Request temperature readings
	transmit(1,	"?FS"); sleep(TIME_BETWEEN_COMMANDS);								// Request status flag
	transmit(1, "?FF"); sleep(TIME_BETWEEN_COMMANDS);								// Request fault flag
	transmit(1, "?CB"); sleep(TIME_BETWEEN_COMMANDS);								// Request absolute hall count
	transmit(1,	"# 10" ); sleep(TIME_BETWEEN_COMMANDS);							// Repeat buffer every 10 ms

	transmit(4, "^CPRI", 1,	0, 0 ); sleep(TIME_BETWEEN_COMMANDS);					// Serial is first and only priority
	transmit(2, "^RWD",	1000 ); sleep(TIME_BETWEEN_COMMANDS);						// One second watchdog
	transmit(2, "^BLFB", 0 ); sleep(TIME_BETWEEN_COMMANDS);						// Use hall sensors as motor feedback
	transmit(2, "^BLSTD", 2 ); sleep(TIME_BETWEEN_COMMANDS);						// Stall detection 500ms@25%
	transmit(2, "^BPOL", 9 ); sleep(TIME_BETWEEN_COMMANDS);						// M12980-1 motor has 9 pole-pairs
	transmit(2, "^OVL",	550 ); sleep(TIME_BETWEEN_COMMANDS);						// Over voltage alerts if >55V
	transmit(2, "^UVL",	485 ); sleep(TIME_BETWEEN_COMMANDS);						// Under voltage alerts if <48.5V
	transmit(2, "^PWMF", 200 ); sleep(TIME_BETWEEN_COMMANDS);						// 20 kHz PWM
	transmit(2, "^THLD", 1 ); sleep(TIME_BETWEEN_COMMANDS);						// Medium sensitive short circuit detection

	if(closed_loop_operation)
	{
		transmit(3, "^MMOD", 1,	1 ); sleep(TIME_BETWEEN_COMMANDS);					// Both channels in closed-loop mode
		transmit(3, "^MMOD", 2,	1 ); sleep(TIME_BETWEEN_COMMANDS);
	}
	else
	{
		transmit(3, "^MMOD", 1,	0 ); sleep(TIME_BETWEEN_COMMANDS);					// Both channels in closed-loop mode
		transmit(3, "^MMOD", 2,	0 ); sleep(TIME_BETWEEN_COMMANDS);
	}
	transmit(3, "^DFC",	1, 0 ); sleep(TIME_BETWEEN_COMMANDS);						// Emergency causes motors to stop
	transmit(3, "^DFC ", 2,	0 ); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "^ALIM", 1, 750 ); sleep(TIME_BETWEEN_COMMANDS);					// Maximum current is 75A for both channels
	transmit(3, "^ALIM", 2, 750 ); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "^CLERD", 1, 0 ); sleep(TIME_BETWEEN_COMMANDS);					// Closed-loop error detection disabled
	transmit(3, "^CLERD", 2, 0 ); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "^ICAP", 1,	anti_windup_percent	); sleep(TIME_BETWEEN_COMMANDS);	// Integral cap anti-windup
	transmit(3, "^ICAP ", 2, anti_windup_percent ); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "^KP", 1, p_gain_ch1 ); sleep(TIME_BETWEEN_COMMANDS);				// Proportional gain for closed-loop control
	transmit(3, "^KP", 2, p_gain_ch2 ); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "^KI", 1, i_gain_ch1 ); sleep(TIME_BETWEEN_COMMANDS);				// Integral gain for closed-loop control
	transmit(3, "^KI", 2, i_gain_ch2 ); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "^KD", 1, d_gain_ch1 ); sleep(TIME_BETWEEN_COMMANDS);				// Differential gain for closed-loop control
	transmit(3, "^KD", 2, d_gain_ch2 ); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "^MAC",	1, max_acceleration	); sleep(TIME_BETWEEN_COMMANDS);		// Maximum allowed acceleration
	transmit(3, "^MAC",	2, max_acceleration	); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "^MDEC", 1,	max_deceleration ); sleep(TIME_BETWEEN_COMMANDS);		// Maximum allowed deceleration
	transmit(3, "^MDEC", 2,	max_deceleration ); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "^MXPF", 1, 95 ); sleep(TIME_BETWEEN_COMMANDS);					// 95% of battery voltage can be applied to motors forward
	transmit(3, "^MXPF", 2,	95 ); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "^MXPR", 1,	95 ); sleep(TIME_BETWEEN_COMMANDS);					// 95% of battery voltage can be applied to motors reverse
	transmit(3, "^MXPR", 2,	95 ); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "^MXRPM", 1,	max_rpm ); sleep(TIME_BETWEEN_COMMANDS);			// Set maximum rounds per minute
	transmit(3, "^MXRPM", 2,	max_rpm );
	sleep(2);

	ROS_INFO("Initialization finished");
	initialised = true;
	idle = false;
}

void hbl2350::onCmdVelCh1(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	last_twist_received_ch1 = ros::Time::now();
	cmd_vel_active = true;
	velocity_ch1 = (int)(msg->twist.linear.x * mps_to_rpm);
}

void hbl2350::onCmdVelCh2(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	last_twist_received_ch2 = ros::Time::now();
	cmd_vel_active = true;
	velocity_ch2 = (int)(msg->twist.linear.x * mps_to_rpm);
}

void hbl2350::onDeadman(const std_msgs::Bool::ConstPtr& msg)
{
	deadman_active = msg->data;
	last_deadman_received = ros::Time::now();
}

/*!Callback for relaying command strings*/
void hbl2350::onCommand(const msgs::serial::ConstPtr& msg)
{
	ROS_WARN("Command callback");
	serial_out.header.stamp = ros::Time::now();
	serial_out.data = msg->data;
	serial_publisher.publish(serial_out);
}

void hbl2350::onTimer(const ros::TimerEvent& e)
{
	if(initialised)
	{
		if(!idle)
		{
			if(deadman_active && cmd_vel_active)
			{
				if((ros::Time::now() - last_deadman_received) > max_time_diff )
				{
					ROS_WARN_THROTTLE(1,"shutting down due to deadman");
					deadman_active = false;
					velocity_ch1 = velocity_ch2 = 0;
				}

				if(( ros::Time::now() - last_twist_received_ch1) > max_time_diff)
				{
					ROS_WARN_THROTTLE(1,"Shutting down due to out of date cmd_vel on ch1");
					velocity_ch1 = velocity_ch2 = 0;
					cmd_vel_active = false;
				}

				if(( ros::Time::now() - last_twist_received_ch2) > max_time_diff)
				{
					ROS_WARN_THROTTLE(1,"Shutting down due to out of date cmd_vel on ch2");
					velocity_ch1 = velocity_ch2 = 0;
					cmd_vel_active = false;
				}

				if(ros::Time::now() - last_serial_msg > max_time_diff * 3)
				{
					ROS_WARN("Lost connection to RoboTeq - shutting down");
					velocity_ch1 = velocity_ch2 = 0;
					initialised = online = false;
				}

				if(deadman_active && cmd_vel_active && initialised)
				{
					int out_ch1 = (velocity_ch1*velocity_max)/max_rpm,
						out_ch2 = (velocity_ch2*velocity_max)/max_rpm;

					if(out_ch1 < - velocity_max)
						out_ch1 = -velocity_max;
					else if(out_ch1 > velocity_max)
						out_ch1 = velocity_max;

					if(out_ch2 < - velocity_max)
						out_ch2 = -velocity_max;
					else if(out_ch2 > velocity_max)
						out_ch2 = velocity_max;

					transmit(3,"!G",1,out_ch1);
					transmit(3,"!G",2,out_ch2);
				}
			}
			else
			{
				transmit(3,"!G",1,0);
				transmit(3,"!G",2,0);
				idle = true;
			}
		}
		else
		{
			if(ros::Time::now() - last_serial_msg < max_time_diff)
				idle = false;
		}
	}
	else
	{
		if(online)
		{
			initController();
		}
		else
		{
			ROS_WARN_THROTTLE(1,"Controller not online");
			transmit(1,"?FID");
		}
	}
}
