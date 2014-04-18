/****************************************************************************
 # FroboMind
 # Licence and description in .hpp file
 ****************************************************************************/
#include "roboteq_hbl1650/hbl1650.hpp"

hbl1650::hbl1650( )
:local_node_handler("~"),global_node_handler()
{
	// One channel operation
	two_channel = false;

	// Setup Channels
	ch1.ch = 1;

	// Register callbacks
	ch1.transmit_cb = new CallbackHandler<hbl1650>(this,&hbl1650::transmit);
	ch1.init_cb = new CallbackHandler<hbl1650>(this,&hbl1650::initController);

	//Motor controller constant open loop max outputmax_output
	ch1.roboteq_max = 1000;

	// Initialise status
	status.cmd_vel_publishing = status.deadman_pressed = status.initialised = status.online = status.responding = false;
	status.emergency_stop = true;

	// Declare variables for parsing parameters
	double max_time_diff_input;
	std::string cmd_vel_ch1_topic, cmd_vel_ch2_topic, serial_tx_topic, serial_rx_topic, command_relay_topic, deadman_topic,
	encoder_ch1_topic, encoder_ch2_topic, power_ch1_topic, power_ch2_topic, status_topic, temperature_topic,velocity_topic,
	propulsion_module_status_topic, propulsion_module_feedback_topic, pid_topic;

	// Parse from parameter server
	local_node_handler.param<std::string>("pid_topic", pid_topic, "/fmInformation/pid");
	local_node_handler.param<std::string>("serial_rx_topic", serial_rx_topic, "/fmCSP/S0_rx");
	local_node_handler.param<std::string>("serial_tx_topic", serial_tx_topic, "/fmCSP/S0_tx");
	local_node_handler.param<std::string>("command_relay_topic", command_relay_topic, "/fmData/command");
	local_node_handler.param<std::string>("deadman_topic", deadman_topic, "/fmHMI/joy");
	local_node_handler.param<std::string>("cmd_vel_ch1_topic", cmd_vel_ch1_topic, "/fmActuators/cmd_vel_ch1");
	local_node_handler.param<std::string>("encoder_ch1_topic", encoder_ch1_topic, "/fmSensors/encoder_ch1");
	local_node_handler.param<std::string>("power_ch1_topic", power_ch1_topic, "/fmSensors/power_ch1");
	local_node_handler.param<std::string>("status_topic", status_topic, "/fmActuators/status");
	local_node_handler.param<std::string>("temperature_topic", temperature_topic, "/fmActuators/temperature");
	//local_node_handler.param<std::string>("velocity_topic", velocity_topic, "/fmActuators/velocity");
	local_node_handler.param<std::string>("propulsion_module_status_topic", propulsion_module_status_topic, "/fmInformation/propulsion_module_status");
	local_node_handler.param<std::string>("propulsion_module_feedback_topic", propulsion_module_feedback_topic, "/fmInformation/propulsion_module_feedback");

	// Init channel parameters
	local_node_handler.param<double>("p_gain", ch1.p_gain, 1);
	local_node_handler.param<double>("i_gain", ch1.i_gain, 0);
	local_node_handler.param<double>("d_gain", ch1.d_gain, 0);
	local_node_handler.param<double>("feed_forward", ch1.ff_gain, 0);
	local_node_handler.param<double>("i_max",ch1.i_max,50);

	local_node_handler.param<double>("/robot_max_velocity",ch1.max_velocity_mps,1.0);
	local_node_handler.param<double>("max_controller_command",ch1.max_output,300);
	if(ch1.max_output > ch1.roboteq_max) ch1.max_output = ch1.roboteq_max;

	local_node_handler.param<double>("mps_to_thrust",ch1.mps_to_thrust,300);

	double tmp;
	local_node_handler.param<double>("ticks_per_meter",tmp,650);
	ch1.ticks_to_meter = 1.0/tmp;

	ch1.time_stamp.last_deadman_received = ros::Time::now();
	ch1.velocity = 0;
	ch1.regulator.set_params(ch1.p_gain , ch1.i_gain , ch1.d_gain, ch1.ff_gain ,ch1.i_max , ch1.roboteq_max);

	// Init position control
	double max_acceleration, max_jerk, brake_zeroband, velocity_tolerance;
	local_node_handler.param<bool>("position_control", ch1.position_control, true);
	local_node_handler.param<double>("robot_max_acceleration", max_acceleration, 0.5);
	local_node_handler.param<double>("robot_max_jerk", max_jerk, 0.5);
	local_node_handler.param<double>("brake_zeroband",brake_zeroband,0.2);
	local_node_handler.param<double>("velocity_tolerance",velocity_tolerance,0.05);

	ch1.position_generator.setMaximumVelocity(ch1.max_velocity_mps);
	ch1.position_generator.setMaximumAcceleration(0.5);
	ch1.position_generator.setMaximumJerk(0.5);
	ch1.position_generator.setBrakeZeroband(0.2);
	ch1.position_generator.setVelocityTolerance(0.05);

	// Init general parameters
	local_node_handler.param<double>("max_time_diff",max_time_diff_input,0.5);
	max_time_diff = ros::Duration(max_time_diff_input);
	last_serial_msg = ros::Time::now();
	local_node_handler.param<bool>("closed_loop_operation", closed_loop_operation, false);

	// Setup publishers
	propulsion_module_status_publisher = local_node_handler.advertise<msgs::PropulsionModuleStatus>( propulsion_module_status_topic,DEFAULT_BUFFER_SIZE );
	setSerialPub( local_node_handler.advertise<msgs::serial>( serial_tx_topic,DEFAULT_BUFFER_SIZE ));
	setEncoderCh1Pub( local_node_handler.advertise<msgs::IntStamped>( encoder_ch1_topic, DEFAULT_BUFFER_SIZE));
	setPowerCh1Pub( local_node_handler.advertise<msgs::IntStamped>( power_ch1_topic, DEFAULT_BUFFER_SIZE));
	setStatusPub( local_node_handler.advertise<msgs::StringStamped>( status_topic, DEFAULT_BUFFER_SIZE));
	ch1.setStatusPub( local_node_handler.advertise<msgs::StringStamped>( status_topic, DEFAULT_BUFFER_SIZE));
	ch1.setVelPub(local_node_handler.advertise<std_msgs::Float64>( velocity_topic, DEFAULT_BUFFER_SIZE));
	ch1.setPropulsionFeedbackPub( local_node_handler.advertise<msgs::PropulsionModuleFeedback>( propulsion_module_feedback_topic, DEFAULT_BUFFER_SIZE));
	ch1.setPidPub( local_node_handler.advertise<msgs::FloatArrayStamped>( pid_topic, DEFAULT_BUFFER_SIZE));
	setTemperaturePub( local_node_handler.advertise<msgs::StringStamped>( temperature_topic, DEFAULT_BUFFER_SIZE));

	// Set up subscribers
	serial_sub = local_node_handler.subscribe<msgs::serial>(serial_rx_topic,DEFAULT_BUFFER_SIZE,&hbl1650::onSerial,this);
	ch1.cmd_vel_sub = local_node_handler.subscribe<geometry_msgs::TwistStamped>(cmd_vel_ch1_topic,DEFAULT_BUFFER_SIZE,&Channel::onCmdVel,&ch1);
	deadman_sub = local_node_handler.subscribe<std_msgs::Bool>(deadman_topic,DEFAULT_BUFFER_SIZE,&hbl1650::onDeadman,this);
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
	ros::Timer t = global_node_handler.createTimer(ros::Duration(0.05),&hbl1650::onTimer,this);
	ros::Timer status_timer = global_node_handler.createTimer(ros::Duration(0.5),&hbl1650::onStatusTimer,this);

	ros::spin();
}

void hbl1650::onStatusTimer(const ros::TimerEvent& event)
{
	propulsion_module_status_message.header.stamp = ros::Time::now();
	propulsion_module_status_message.voltage = v2/10.0;
	propulsion_module_status_message.current = ba1/10.0;
	propulsion_module_status_message.power = (v2*ba1)/100.0;
	propulsion_module_status_publisher.publish(propulsion_module_status_message);

	transmit(1,	"# C"); sleep(TIME_BETWEEN_COMMANDS);
	transmit(1,	"?V"); sleep(TIME_BETWEEN_COMMANDS);								// Request power readings
	transmit(1,	"?BA"); sleep(TIME_BETWEEN_COMMANDS);								// Request voltage readings
	transmit(1,	"?T"); sleep(TIME_BETWEEN_COMMANDS);								// Request temperature readings
	transmit(1,	"?FS"); sleep(TIME_BETWEEN_COMMANDS);								// Request status flag
	transmit(1, "?FF"); sleep(TIME_BETWEEN_COMMANDS);								// Request fault flag
	transmit(1, "?CB"); sleep(TIME_BETWEEN_COMMANDS);								// Request absolute hall count
	transmit(1,	"# 20" ); sleep(TIME_BETWEEN_COMMANDS);							    // Repeat buffer every 50 ms
}

void hbl1650::updateStatus(void)
{
	// Update time based status variables.
	status.deadman_pressed = ((ros::Time::now() - ch1.time_stamp.last_deadman_received) < max_time_diff);
	status.cmd_vel_publishing = ( (ros::Time::now() - ch1.time_stamp.last_twist_received) < max_time_diff);
	status.responding = ((ros::Time::now() - last_serial_msg) < max_time_diff);
}

void hbl1650::initController(std::string config)
{
	/*
	 * Temporary simple implementation. Should in time be implemented as a config file parser. In time...
	 */
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
	transmit(1,	"# 10" ); sleep(TIME_BETWEEN_COMMANDS);							    // Repeat buffer every 10 ms
	sleep(2);

	ROS_INFO("Initialization finished");
	status.initialised = true;
}

