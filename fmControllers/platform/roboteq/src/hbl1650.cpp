#include "roboteq/hbl1650.hpp"

hbl1650::hbl1650( )
:local_node_handler("~"),global_node_handler()
{

	// Variables for parsing parameters
	std::string
	cmd_vel_ch1_topic,
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
	deadman_pressed = false;
	last_deadman_received = ros::Time::now();
	initialised = false;
	//online = false;
	emergency_stop = true;
	last_serial_msg = ros::Time::now();
	cmd_vel_publishing = false;
	controller_responding = true;
	velocity = 0;

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

	local_node_handler.param<int>("p_gain",	p_gain,	1);
	local_node_handler.param<int>("i_gain",	i_gain,	0);
	local_node_handler.param<int>("d_gain",	d_gain,	0);
	local_node_handler.param<bool>("closed_loop_operation",	closed_loop_operation,	false);

	local_node_handler.param<double>("max_time_diff",max_time_diff_input,0.5);
	max_time_diff = ros::Duration(max_time_diff_input);
	local_node_handler.param<double>("mps_to_rpm",mps_to_rpm,5);

	local_node_handler.param<int>("max_acceleration",max_acceleration,20000);
	local_node_handler.param<int>("max_deceleration",max_deceleration,20000);
	local_node_handler.param<int>("max_rpm",max_rpm,4000);
	local_node_handler.param<int>("anti_windup_percent",anti_windup_percent,50);
	velocity_max = 1000; //Motor controller constant

	// Set publisher topics
	setSerialPub(local_node_handler.advertise<msgs::serial>(serial_tx_topic,10) );
	setEncoderPub(local_node_handler.advertise<msgs::IntStamped>(encoder_ch1_topic,10) );
	setPowerPub(local_node_handler.advertise<msgs::IntStamped>(power_ch1_topic,10));
	setStatusPub(local_node_handler.advertise<msgs::StringStamped>(status_topic,10) );
	//	setTemperaturePub(	local_node_handler.advertise<fmMsgs::StringStamped>(temperature_topic,	10) );

	// Set subscriber topics
	serial_sub = local_node_handler.subscribe<msgs::serial>(serial_rx_topic,10,&hbl1650::onSerial,this);
	command_relay_sub = local_node_handler.subscribe<msgs::serial>(command_relay_topic,10,&hbl1650::onCommand,this);
	cmd_vel_sub = local_node_handler.subscribe<geometry_msgs::TwistStamped>(cmd_vel_ch1_topic,10,&hbl1650::onCmdVel,this);
	deadman_sub = local_node_handler.subscribe<std_msgs::Bool>(deadman_topic,10,&hbl1650::onDeadman,this);

}

void hbl1650::spin(void)
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
	ros::Timer t = global_node_handler.createTimer(ros::Duration(0.1),&hbl1650::onTimer,this);

	ros::spin();
}

void hbl1650::initController(void)
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

/*	transmit(4, "^CPRI", 1,	0, 0 ); sleep(TIME_BETWEEN_COMMANDS);					// Serial is first and only priority
	transmit(2, "^RWD",	1000 ); sleep(TIME_BETWEEN_COMMANDS);						// One second watchdog

	transmit(3, "^BLFB", 1, 1 ); sleep(TIME_BETWEEN_COMMANDS);						// Use hall sensors as motor feedback
	transmit(3, "^BLFB", 2, 1 ); sleep(TIME_BETWEEN_COMMANDS);						// Use hall sensors as motor feedback

	transmit(3, "^BLSTD", 1, 2 ); sleep(TIME_BETWEEN_COMMANDS);						// Stall detection 500ms@25%
	transmit(3, "^BLSTD", 2, 2 ); sleep(TIME_BETWEEN_COMMANDS);						// Stall detection 500ms@25%

	transmit(3, "^BPOL", 1, 9 ); sleep(TIME_BETWEEN_COMMANDS);						// M12980-1 motor has 9 pole-pairs
	transmit(3, "^BPOL", 2, 9 ); sleep(TIME_BETWEEN_COMMANDS);						// M12980-1 motor has 9 pole-pairs

	transmit(2, "^OVL",	550 ); sleep(TIME_BETWEEN_COMMANDS);						// Over voltage alerts if >55V
	transmit(2, "^UVL",	480 ); sleep(TIME_BETWEEN_COMMANDS);						// Under voltage alerts if <48.5V
	transmit(2, "^PWMF", 200 ); sleep(TIME_BETWEEN_COMMANDS);						// 20 kHz PWM
	transmit(2, "^THLD", 1 ); sleep(TIME_BETWEEN_COMMANDS);						// Medium sensitive short circuit detection

	if(closed_loop_operation)
	{
		transmit(3, "^MMOD", 1,	1 ); sleep(TIME_BETWEEN_COMMANDS);					// Both channels in closed-loop mode
		transmit(3, "^MMOD", 2,	1 ); sleep(TIME_BETWEEN_COMMANDS);
		transmit(3, "^ICAP", 1,	anti_windup_percent	); sleep(TIME_BETWEEN_COMMANDS);	// Integral cap anti-windup
		transmit(3, "^ICAP ", 2, anti_windup_percent ); sleep(TIME_BETWEEN_COMMANDS);
		transmit(3, "^KP", 1, p_gain_ch1 ); sleep(TIME_BETWEEN_COMMANDS);				// Proportional gain for closed-loop control
		transmit(3, "^KP", 2, p_gain_ch2 ); sleep(TIME_BETWEEN_COMMANDS);
		transmit(3, "^KI", 1, i_gain_ch1 ); sleep(TIME_BETWEEN_COMMANDS);				// Integral gain for closed-loop control
		transmit(3, "^KI", 2, i_gain_ch2 ); sleep(TIME_BETWEEN_COMMANDS);
		transmit(3, "^KD", 1, d_gain_ch1 ); sleep(TIME_BETWEEN_COMMANDS);				// Differential gain for closed-loop control
		transmit(3, "^KD", 2, d_gain_ch2 ); sleep(TIME_BETWEEN_COMMANDS);
	}
	else
	{
		transmit(3, "^MMOD", 1,	0 ); sleep(TIME_BETWEEN_COMMANDS);					// Both channels in open-loop mode
		transmit(3, "^MMOD", 2,	0 ); sleep(TIME_BETWEEN_COMMANDS);
	}
	transmit(3, "^DFC",	1, 0 ); sleep(TIME_BETWEEN_COMMANDS);						// Emergency causes motors to stop
	transmit(3, "^DFC ", 2,	0 ); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "^ALIM", 1, 400 ); sleep(TIME_BETWEEN_COMMANDS);					// Maximum current is 40A for both channels
	transmit(3, "^ALIM", 2, 400 ); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "^CLERD", 1, 0 ); sleep(TIME_BETWEEN_COMMANDS);					// Closed-loop error detection disabled
	transmit(3, "^CLERD", 2, 0 ); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "^MAC",	1, max_acceleration	); sleep(TIME_BETWEEN_COMMANDS);		// Maximum allowed acceleration
	transmit(3, "^MAC",	2, max_acceleration	); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "^MDEC", 1,	max_deceleration ); sleep(TIME_BETWEEN_COMMANDS);		// Maximum allowed deceleration
	transmit(3, "^MDEC", 2,	max_deceleration ); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "!AC",	1, max_acceleration	); sleep(TIME_BETWEEN_COMMANDS);		// Maximum allowed acceleration
	transmit(3, "!AC",	2, max_acceleration	); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "!DC", 1,	max_deceleration ); sleep(TIME_BETWEEN_COMMANDS);		// Maximum allowed deceleration
	transmit(3, "!DC", 2,	max_deceleration ); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "^MXPF", 1, 95 ); sleep(TIME_BETWEEN_COMMANDS);					// 95% of battery voltage can be applied to motors forward
	transmit(3, "^MXPF", 2,	95 ); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "^MXPR", 1,	95 ); sleep(TIME_BETWEEN_COMMANDS);					// 95% of battery voltage can be applied to motors reverse
	transmit(3, "^MXPR", 2,	95 ); sleep(TIME_BETWEEN_COMMANDS);
	transmit(3, "^MXRPM", 1,	max_rpm ); sleep(TIME_BETWEEN_COMMANDS);			// Set maximum rounds per minute
	transmit(3, "^MXRPM", 2,	max_rpm );
*/	sleep(2);

	ROS_INFO("Initialization finished");
	initialised = true;
	controller_responding = false;
}

void hbl1650::onCmdVel(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	last_twist_received = ros::Time::now();
	velocity = (int)(msg->twist.linear.x * mps_to_rpm);
}

void hbl1650::onDeadman(const std_msgs::Bool::ConstPtr& msg)
{
	deadman_pressed = msg->data;
	if(deadman_pressed)
		last_deadman_received = ros::Time::now();
}

/*!Callback for relaying command strings*/
void hbl1650::onCommand(const msgs::serial::ConstPtr& msg)
{
	ROS_WARN("Command callback");
	serial_out.header.stamp = ros::Time::now();
	serial_out.data = msg->data;
	serial_publisher.publish(serial_out);
}

void hbl1650::onTimer(const ros::TimerEvent& e)
{
	/* Update state variables */
	deadman_pressed = ((ros::Time::now() - last_deadman_received) < max_time_diff);
	cmd_vel_publishing = ( (ros::Time::now() - last_twist_received) < max_time_diff);
	controller_responding = ((ros::Time::now() - last_serial_msg) < max_time_diff);

	std::stringstream ss;

	if(1/*online*/) /* is set when controller answers to FID request */
	{
		ss << "controller_online ";
		if(initialised) /* is set when initController function completes */
		{
			ss << "controller_initialised ";
			if(controller_responding) /* is set if the controller publishes serial messages */
			{
				ss << "controller_responding ";
				if(cmd_vel_publishing) /* is set if someone publishes twist messages */
				{
					ss << "cmd_vel_publishing ";
					if(deadman_pressed) /* is set if someone publishes true on deadman topic */
					{
						if(emergency_stop)
						{
							transmit(1,"!MG");
							emergency_stop = false;
						}

						ss << "deadman_pressed ";
						/* All is good - send speed */
						int out_ch1 = (velocity*velocity_max)/max_rpm;

						if(out_ch1 < - velocity_max)
							out_ch1 = -velocity_max;
						else if(out_ch1 > velocity_max)
							out_ch1 = velocity_max;

						transmit(2,"!G",out_ch1);
					}
					else /* deadman not pressed */
					{
//						ROS_INFO("%s: Deadman button is not pressed",ros::this_node::getName().c_str());
						/* Set speeds to 0 */
						transmit(1,"!EX");
						emergency_stop = true;
						transmit(2,"!G",0);
					}
				}
				else /* Cmd_vel is not publishing */
				{
//					ROS_INFO("%s: Cmd_vel is not publishing",ros::this_node::getName().c_str());
					/* Set speeds to 0 */
					transmit(1,"!EX");
					emergency_stop = true;
					transmit(2,"!G",0);
				}
			}
			else /* controller is not responding */
			{
				ROS_INFO("%s: Controller is not responding",ros::this_node::getName().c_str());
				transmit(1,"?FID");
			}
		}
		else /* Controller is not initialised */
		{
			ROS_INFO("%s: Controller is not initialised",ros::this_node::getName().c_str());
			initController();
		}
	}
	else /* controller is not online */
	{
		ROS_INFO("%s: Controller is not yet online",ros::this_node::getName().c_str());
		transmit(1,"?FID");
	}
	status_out.header.stamp = ros::Time::now();
	status_out.data = ss.str();
	status_publisher.publish(status_out);
}
