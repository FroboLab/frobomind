#include "roboteq/hbl2350.hpp"

Channel::Channel( )
{
//	deadman_pressed = cmd_vel_publishing = initialised = controller_responding =false;
//	emergency_stop = true;
//	mps_to_rpm = p_gain = i_gain = d_gain = velocity = anti_windup_percent = velocity_max = max_rpm = max_acceleration = max_deceleration = 0;

}

void Channel::onHallFeedback(ros::Time time, int feedback)
{
	message.hall.header.stamp = time;
	message.hall.data = feedback;
	publisher.hall.publish(message.hall);
}

void Channel::onPowerFeedback(ros::Time time, int feedback)
{
	message.power.header.stamp = time;
	message.power.data = feedback;
	publisher.power.publish(message.power);
}

void Channel::onTemperatureFeedback(ros::Time time, int feedback)
{
	message.temperature.header.stamp = time;
	message.temperature.data = feedback;
	publisher.temperature.publish(message.temperature);
}

//void Channel::CmdVel(const geometry_msgs::TwistStamped::ConstPtr& msg)
//{
//	last_twist_received = ros::Time::now();
//	velocity = msg->twist.linear.x;
//}
//
//
//void Channel::Deadman(const std_msgs::Bool::ConstPtr& msg)
//{
//	deadman_pressed = msg->data;
//	if(deadman_pressed)
//		last_deadman_received = ros::Time::now();
//}
//
//void Channel::hall_feedback(ros::Time time, int fb1)
//{
//	// TODO: use registered callback instead
//	encoder_out.header.stamp = time;
//
//	encoder_out.data = cb1;
//	encoder_ch1_publisher.publish(encoder_out);
//}
//
//void Channel::power_feedback(ros::Time time, int fb1)
//{
//	// TODO: use registered callback instead
//	power_out.header.stamp = time;
//	power_out.data = fb1;
//	power_ch1_publisher.publish(power_out);
//}
//
//
//void Channel::Timer(const ros::TimerEvent& e)
//{
//	/* Update state variables */
//	deadman_pressed = ((ros::Time::now() - last_deadman_received) < max_time_diff);
//	cmd_vel_publishing = ( (ros::Time::now() - last_twist_received) < max_time_diff);
//	controller_responding = ((ros::Time::now() - last_serial_msg) < max_time_diff);
//
//	std::stringstream ss;
//
//	if(online) /* is set when contrChannel::oller answers to FID request */
//	{
//		ss << "controller_online ";
//		if(initialised) /* is set when init function completes */
//		{
//			ss << "controller_initialised ";
//			if(controller_responding) /* is set if the controller publishes serial messages */
//			{
//				ss << "controller_responding ";
//				if(cmd_vel_publishing) /* is set if someone publishes twist messages */
//				{
//					ss << "cmd_vel_publishing ";
//					if(deadman_pressed) /* is set if someone publishes true on deadman topic */
//					{
//						if(emergency_stop)
//						{
//							transmit(1,"!MG");
//							emergency_stop = false;
//						}
//
//						ss << "deadman_pressed ";
//						/* Get new output */
//						double setpoint = regulator.output_from_input(velocity, ticks_to_mps(encoder), (ros::Time::now() - last_reg_time).to_sec());
//
//						// Convert to rpm
//						setpoint *= mps_to_rpm;
//
//						//Convert to RoboTeQ
//						int out = (setpoint*velocity_max)/max_rpm;
//
//						if(out < - velocity_max)
//							out = -Channel::velocity_max;
//						else if(out > velocity_max)
//							out = velocity_max;
//
//						transmit(3,"!G",1,out);
//					}
//					else /* deadman not pressed */
//					{
////						ROS_INFO("%s: Deadman button is not pressed",ros::this_node::getName().c_str());
//						/* Set speeds to 0 */
//						transmit(1,"!EX");
//						emergency_stop = true;
//						transmit(3,"!G",1,0);
//					}
//				}
//				else /* Cmd_vel is not publishing */
//				{
////					ROS_INFO("%s: Cmd_vel is not publishing",ros::this_node::getName().c_str());
//					/* Set speeds to 0 */
//					transmit(1,"!EX");
//					emergency_stop = true;
//					transmit(3,"!G",1,0);
//				}
//			}
//			else /* controller is not responding */
//			{
//				ROS_INFO("%s: Controller is not responding",ros::this_node::getName().c_str());
//				transmit(1,"?FID");
//			}
//		}
//		else /* Controller is not initialised */
//		{
//			ROS_INFO("%s: ControlleChannel::r is not initialised",ros::this_node::getName().c_str());
//			initialised = initController();
//		}
//	}
//	else /* controller is not online */
//	{
//		ROS_INFO("%s: Controller is not yet online",ros::this_node::getName().c_str());
//		transmit(1,"?FID");
//	}
//	status_out.header.stamp = ros::Time::now();
//	status_out.data = ss.str();
//	status_publisher.publish(status_out);
//}
//
////Temporary hack work only on pichi...
//bool Channel::initController( void )
//{
////	ROS_INFO("Initializing...");
////	sleep(1);
//	transmit(2,	"^ECHOF", 1 ); sleep(TIME_BETWEEN_COMMANDS);						// Echo is disabled
//
//	transmit(1,	"# C"); sleep(TIME_BETWEEN_COMMANDS);								// Clear buffer
//	transmit(1,	"?P"); sleep(TIME_BETWEEN_COMMANDS);								// Request power readings
//	transmit(1,	"?V"); sleep(TIME_BETWEEN_COMMANDS);								// Request voltage readings
//	transmit(1,	"?T"); sleep(TIME_BETWEEN_COMMANDS);								// Request temperature readings
//	transmit(1,	"?FS"); sleep(TIME_BETWEEN_COMMANDS);								// Request status flag
//	transmit(1, "?FF"); sleep(TIME_BETWEEN_COMMANDS);								// Request fault flag
//	transmit(1, "?CB"); sleep(TIME_BETWEEN_COMMANDS);								// Request absolute hall count
////	controller->transmit(1, "?CBR"); sleep(TIME_BETWEEN_COMMANDS);
//	transmit(1,	"# 10" ); sleep(TIME_BETWEEN_COMMANDS);							// Repeat buffer every 10 ms
//
///*	controller->transmit(4, "^CPRI", 1,	0, 0 ); sleep(TIME_BETWEEN_COMMANDS);					// Serial is first and only priority
//	controller->transmit(2, "^RWD",	1000 ); sleep(TIME_BETWEEN_COMMANDS);						// One second watchdog
//
//	controller->transmit(3, "^BLFB", 1, 1 ); sleep(TIME_BETWEEN_COMMANDS);						// Use hall sensors as motor feedback
//	controller->transmit(3, "^BLFB", 2, 1 ); sleep(TIME_BETWEEN_COMMANDS);						// Use hall sensors as motor feedback
//
//	controller->transmit(3, "^BLSTD", 1, 2 ); sleep(TIME_BETWEEN_COMMANDS);						// Stall detection 500ms@25%
//	controller->transmit(3, "^BLSTD", 2, 2 ); sleep(TIME_BETWEEN_COMMANDS);						// Stall detection 500ms@25%
//
//	controller->transmit(3, "^BPOL", 1, 9 ); sleep(TIME_BETWEEN_COMMANDS);						// M12980-1 motor has 9 pole-pairs
//	controller->transmit(3, "^BPOL", 2, 9 ); sleep(TIME_BETWEEN_COMMANDS);						// M12980-1 motor has 9 pole-pairs
//
//	controller->transmit(2, "^OVL",	550 ); sleep(TIME_BETWEEN_COMMANDS);						// Over voltage alerts if >55V
//	controller->transmit(2, "^UVL",	480 ); sleep(TIME_BETWEEN_COMMANDS);						// Under voltage alerts if <48.5V
//	controller->transmit(2, "^PWMF", 200 ); sleep(TIME_BETWEEN_COMMANDS);						// 20 kHz PWM
//	controller->transmit(2, "^THLD", 1 ); sleep(TIME_BETWEEN_COMMANDS);						// Medium sensitive short circuit detection
//
//	if(closed_loop_operation)
//	{
//		controller->transmit(3, "^MMODint main (int argc, char** argv)
//{
//	ros::void RoboTeQ::hall_feedback(ros::Time time , int fb1 , int fb2)
//{
//	encoder_out.header.stamp = time;
//
//	encoder_out.data = fb1;
//	encoder_ch1_publisher.publish(encoder_out);
//
//	encoder_out.data = fb2;
//	encoder_ch2_publisher.publish(encoder_out);
//}
//
//void RoboTeQ::hall_feedback(ros::Time time, int fb1)
//{
//	encoder_out.header.stamp = time;
//
//	encoder_out.data = cb1;
//	encoder_ch1_publisher.publish(encoder_out);
//}
//
//void RoboTeQ::power_feedback(ros::Time time , int fb1 , int fb2)
//{
//	power_out.header.stamp = time;
//	power_out.data = fb1;
//	power_ch1_publisher.publish(power_out);
//
//	power_out.data = fb2;
//	power_ch2_publisher.publish(power_out);
//}
//
//void RoboTeQ::power_feedback(ros::Time time, int fb1)
//{
//	power_out.header.stamp = time;
//	power_out.data = fb1;
//	power_ch1_publisher.publish(power_out);
//}init(argc,argv,"roboteq_controller");
//	hbl2350 controller(&initController);
//	controller.spin();
//
//	return 0;
//}", 1,	1 ); sleep(TIME_BETWEEN_COMMANDS);					// Both channels in closed-loop mode
//		controller->transmit(3, "^MMOD", 2,	1 ); sleep(TIME_BETWEEN_COMMANDS);
//		controller->transmit(3, "^ICAP", 1,	anti_windup_percent	); sleep(TIME_BETWEEN_COMMANDS);	// Integral cap anti-windup
//		controller->transmit(3, "^ICAP ", 2, anti_windup_percent ); sleep(TIME_BETWEEN_COMMANDS);
//		controller->transmit(3, "^KP", 1, p_gain_ch1 ); sleep(TIME_BETWEEN_COMMANDS);				// Proportional gain for closed-loop control
//		controller->transmit(3, "^KP", 2, p_gain_ch2 ); sleep(TIME_BETWEEN_COMMANDS);
//		controller->transmit(3, "^KI", 1, ch1.i_gain ); sleep(TIME_BETWEEN_COMMANDS);				// Integral gain for closed-loop control
//		controller->tint nransmit(3, "^KI", 2, i_gain_ch2 ); sleep(TIME_BETWEEN_COMMANDS);
//		controller->transmit(3, "^KD", 1, d_gain_ch1 ); sleep(TIME_BETWEEN_COMMANDS);				// Differential gain for closed-loop control
//		controller->transmit(3, "^KD", 2, d_gain_ch2 ); sleep(TIME_BETWEEN_COMMANDS);
//	}
//	else
//	{
//		controller->transmit(3, "^MMOD", 1,	0 ); sleep(TIME_BETWEEN_COMMANDS);					// Both channels in open-loop mode
//		controller->transmit(3, "^MMOD", 2,	0 ); sleep(TIME_BETWEEN_COMMANDS);
//	}
//	controller->transmit(3, "^DFC",	1, 0 ); sleep(TIME_BETWEEN_COMMANDS);						// Emergency causes motors to stop
//	controller->transmit(3, "^DFC ", 2,	0 ); sleep(TIME_BETWEEN_COMMANDS);
//	controller->transmit(3, "^ALIM", 1, 400 ); sleep(TIME_BETWEEN_COMMANDS);					// Maximum current is 40A for both channels
//	controller->transmit(3, "^ALIM", 2, 400 ); sleep(TIME_BETWEEN_COMMANDS);
//	controller->transmit(3, "^CLERD", 1, 0 ); sleep(TIME_BETWEEN_COMMANDS);					// Closed-loop error detection disabled
//	controller->transmit(3, "^CLERD", 2, 0 ); sleep(TIME_BETWEEN_COMMANDS);
//	controller->transmit(3, "^MAC",	1, max_acceleration	); sleep(TIME_BETWEEN_COMMANDS);		// Maximum allowed acceleration
//	controller->transmit(3, "^MAC",	2, max_acceleration	); sleep(TIME_BETWEEN_COMMANDS);
//	controller->transmit(3, "^MDEC", 1,	max_deceleration ); sleep(TIME_BETWEEN_COMMANDS);		// Maximum allowed deceleration
//	controller->transmit(3, "^MDEC", 2,	max_deceleration ); sleep(TIME_BETWEEN_COMMANDS);
//	controller->transmit(3, "!AC",	1, max_acceleration	); sleep(TIME_BETWEEN_COMMANDS);		// Maximum allowed acceleration
//	controller->transmit(3, "!AC",	2, max_acceleration	); sleep(TIME_BETWEEN_COMMANDS);
//	controller->transmit(3, "!DC", 1,	max_deceleration ); sleep(TIME_BETWEEN_COMMANDS);		// Maximum allowed deceleration
//	controller->transmit(3, "!DC", 2,	max_deceleration ); sleep(TIME_BETWEEN_COMMANDS);
//	controller->transmit(3, "^MXPF", 1, 95 ); sleep(TIME_BETWEEN_COMMANDS);					// 95% of battery voltage can be applied to motors forward
//	controller->transmit(3, "^MXPF", 2,	95 ); sleep(TIME_BETWEEN_COMMANDS);
//	controller->transmit(3, "^MXPR", 1,	95 ); sleep(TIME_BETWEEN_COMMANDS);					// 95% of battery voltage can be applied to motors reverse
//	controller->transmit(3, "^MXPR", 2,	95 ); sleep(TIME_BETWEEN_COMMANDS);	void setInit(void(init)(RoboTeQ::*)){ch1.initController = ch2.initController = init;}
//	controller->transmit(3, "^MXRPM", 1,	max_rpm ); sleep(TIME_BETWEEN_COMMANDS);			// Set maximum rounds per minute
//	controller->transmit(3, "^MXRPM", 2,	max_rpm );
//*/	sleep(2);
//
//	ROS_INFO("Initialization finished");
//	return true;
//}
