/****************************************************************************
 # FroboMind frobit_interface.cpp
 # Copyright (c) 2011-2013, author Leon Bonde Larsen <leon@bondelarsen.dk>
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 #	* Redistributions of source code must retain the above copyright
 #  	notice, this list of conditions and the following disclaimer.
 #	* Redistributions in binary form must reproduce the above copyright
 #  	notice, this list of conditions and the following disclaimer in the
 #  	documentation and/or other materials provided with the distribution.
 #	* Neither the name FroboMind nor the
 #  	names of its contributors may be used to endorse or promote products
 #  	derived from this software without specific prior written permission.
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
 ****************************************************************************
 # Change log:
 # 08-08-2013 Leon: Implemented new protocol
 ****************************************************************************/
#include "frobit_interface.hpp"

FrobitInterface::FrobitInterface() :
local_node_handler("~"), global_node_handler()
{
	left_vel = 0;
	right_vel = 0;
	meters_to_ticks = 0;

	active = false;
	deadman = false;

	last_deadman_received = ros::Time::now();
}
;

void FrobitInterface::makeItSpin(void)
{
	// Get global parameters
	global_node_handler.param<double>("/robot_max_velocity", parameters.max_velocity, 1);
	global_node_handler.param<double>("/diff_steer_wheel_radius", parameters.wheel_radius, 1);
	global_node_handler.param<double>("/diff_steer_wheel_ticks_per_rev", parameters.ticks_pr_rev, 360);

	// Get local parameters
	local_node_handler.param<std::string>("nmea_sub", topics.nmea_sub, "/fmData/nmea_from_frobit");
	local_node_handler.param<std::string>("nmea_pub", topics.nmea_pub, "/fmActuators/duty");
	local_node_handler.param<std::string>("deadman_sub", topics.deadman, "/fmSignals/deadman");
	local_node_handler.param<std::string>("cmd_vel_left_sub", topics.cmd_vel_left, "/fmDecision/twist");
	local_node_handler.param<std::string>("cmd_vel_right_sub", topics.cmd_vel_right, "/fmDecision/twist");
	local_node_handler.param<std::string>("encoder_left_pub", topics.encoder_left, "/fmInformation/encoder_left");
	local_node_handler.param<std::string>("encoder_right_pub", topics.encoder_right, "/fmInformation/encoder_right");
	local_node_handler.param<std::string>("adc_data_pub", topics.adc_data, "/fmInformation/adc_data");
	local_node_handler.param<bool>("castor_front", parameters.castor_front, true);
	local_node_handler.param<double>("cmd_vel_timeout", parameters.timeout, 1);
	local_node_handler.param<double>("nmea_to_frobit_interval", parameters.interval, 0.1);

	// Calculate conversion factors
	meters_to_ticks = parameters.ticks_pr_rev / (2 * M_PI * parameters.wheel_radius);

	// Init subscribers, publishers and timers
	subscribers.cmd_vel_left = global_node_handler.subscribe<geometry_msgs::TwistStamped>(topics.cmd_vel_left, 2, &FrobitInterface::on_vel_msg_left, this);
	subscribers.cmd_vel_right = global_node_handler.subscribe<geometry_msgs::TwistStamped>(topics.cmd_vel_right, 2, &FrobitInterface::on_vel_msg_right, this);
	subscribers.deadman = global_node_handler.subscribe<std_msgs::Bool>(topics.deadman, 2, &FrobitInterface::on_deadman,this);
	subscribers.nmea = global_node_handler.subscribe<msgs::nmea>(topics.nmea_sub, 2, &FrobitInterface::on_nmea, this);

	publishers.nmea = global_node_handler.advertise<msgs::nmea>(topics.nmea_pub, 1);
	publishers.encoder_left = global_node_handler.advertise<msgs::IntStamped>(topics.encoder_left, 1);
	publishers.encoder_right = global_node_handler.advertise<msgs::IntStamped>(topics.encoder_right, 1);
	publishers.data = global_node_handler.advertise<msgs::IntArrayStamped>(topics.adc_data, 1);

	ros::Timer t1 = global_node_handler.createTimer(ros::Duration(parameters.interval), &FrobitInterface::on_timer, this);

	ros::spin();
}

void FrobitInterface::on_vel_msg_left(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	messages.cmd_vel_left = *msg;
}

void FrobitInterface::on_vel_msg_right(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	messages.cmd_vel_right = *msg;
}

void FrobitInterface::on_deadman(const std_msgs::Bool::ConstPtr& msg)
{
	deadman = msg->data;
	last_deadman_received = ros::Time::now();
}

void FrobitInterface::on_nmea(const msgs::nmea::ConstPtr& msg)
{
	// Sort out messages according to NMEA identifier

	if(msg->valid)
	{
		if(msg->type == "PFBHI")
		{
			if(msg->data.size() == PFBHI_DATA_LENGTH)
				handle_startup_message(msg);
			else
				handle_corrupt_data(msg);
		}
		else if(msg->type == "PFBST")
		{
			if(msg->data.size() == PFBST_DATA_LENGTH)
				handle_status_message(msg);
			else
				handle_corrupt_data(msg);
		}
		else if(msg->type == "PFDAT")
		{
			handle_data_message(msg);
		}
		else
			handle_unknown_type(msg);
	}
	else
		handle_invalid_message(msg);
}

void FrobitInterface::handle_startup_message(const msgs::nmea::ConstPtr& msg)
{}

void FrobitInterface::handle_status_message(const msgs::nmea::ConstPtr& msg)
{
	messages.encoder.header.stamp = msg->header.stamp;
	messages.encoder.data = boost::lexical_cast<int>(msg->data.at(1));
	publishers.encoder_left.publish(messages.encoder);

	messages.encoder.data = boost::lexical_cast<int>(msg->data.at(2));
	publishers.encoder_right.publish(messages.encoder);
}

void FrobitInterface::handle_data_message(const msgs::nmea::ConstPtr& msg)
{
	messages.data.header.stamp = msg->header.stamp;
	messages.data.data.clear();
	for(int i = 0 ; i < msg->data.size()-1 ; i++)
	{
		int received = boost::lexical_cast<int>(msg->data.at(i));
		messages.data.data.push_back(received);
	}
	publishers.data.publish(messages.data);
}

void FrobitInterface::handle_control_message()
{
	// TODO: Must implement $PFBSP,voltage_min*CS<cr><lf>
}

void FrobitInterface::handle_communication_parameters_message()
{
	// TODO: Must implement $PFBCP,pfbst_interval, pfbct_wd_timeout*CS<cr><lf>
}

void FrobitInterface::handle_wheel_parameters_message()
{
	// 	TODO: Must implement $PFBWP,pid_enable,pid_interval,Kp_L,Ki_L,Kd_L,Kp_R,Ki_R,Kd_R*CS<cr><lf>
}

void FrobitInterface::handle_corrupt_data(const msgs::nmea::ConstPtr& msg)
{
	ROS_ERROR("%s: Received NMEA message with corrupted data.",ros::this_node::getName().c_str());
	std::cout << *msg;
}

void FrobitInterface::handle_unknown_type(const msgs::nmea::ConstPtr& msg)
{
	ROS_ERROR("%s: Received NMEA message with unknown id.",ros::this_node::getName().c_str());
	std::cout << *msg;
}

void FrobitInterface::handle_invalid_message(const msgs::nmea::ConstPtr& msg)
{
	ROS_ERROR("%s: Received NMEA message with invalid checksum.",ros::this_node::getName().c_str());
	std::cout << *msg;
}

bool FrobitInterface::all_ok(void)
{
	active = true;
	if ((ros::Time::now() - messages.cmd_vel_left.header.stamp).toSec() > parameters.timeout)
	{
		ROS_WARN_THROTTLE(1, "Time for left cmd_vel is out of date");
		active = false;
	}

	if ((ros::Time::now() - messages.cmd_vel_right.header.stamp).toSec() > parameters.timeout)
	{
		ROS_WARN_THROTTLE(1, "Time for right cmd_vel is out of date");
		active = false;
	}

	if (ros::Time::now() > last_deadman_received + ros::Duration(0.2))
	{
		deadman = false;
	}
	return active && deadman;
}

double FrobitInterface::correct_to_max_velocity(double vel)
{
    double corr_max_velocity = parameters.max_velocity + abs(abs(left_vel)-abs(right_vel))/2;

    if (vel > corr_max_velocity)
      vel = corr_max_velocity;
    else if (vel < -corr_max_velocity)
      vel = -corr_max_velocity;

	return vel;
}

void FrobitInterface::on_timer(const ros::TimerEvent& e)
{
	if (all_ok())
	{
		if (parameters.castor_front)
		{
			left_vel = messages.cmd_vel_left.twist.linear.x;
			right_vel = messages.cmd_vel_right.twist.linear.x;
		}
		else
		{
			left_vel = messages.cmd_vel_right.twist.linear.x * -1;
			right_vel = messages.cmd_vel_left.twist.linear.x * -1;
		}

		//correct to max velocity and convert from [m/s] to [ticks/s]
		std::cout << "Vel left:" << left_vel << " Vel right:"<< right_vel << std::endl;
		left_vel = correct_to_max_velocity(left_vel) * meters_to_ticks;
		right_vel = correct_to_max_velocity(right_vel) * meters_to_ticks;

	}
	else
	{
		left_vel = 0;
		right_vel = 0;
	}

	//build message
	messages.motor_command.header.stamp = ros::Time::now();
	messages.motor_command.type = "PFBCT";
	messages.motor_command.data.clear();
	messages.motor_command.data.push_back(boost::lexical_cast<std::string>((int)left_vel));
	messages.motor_command.data.push_back(boost::lexical_cast<std::string>((int)right_vel));

	//publish message
	publishers.nmea.publish(messages.motor_command);
}

