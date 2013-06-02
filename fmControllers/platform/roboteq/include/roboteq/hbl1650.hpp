/****************************************************************************
 # FroboMind
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
 #
 #
 #
 ****************************************************************************/
#ifndef HBL1650_HPP_
#define HBL1650_HPP_

#define TIME_BETWEEN_COMMANDS 0.2

#include <ros/ros.h>
#include "roboteq/roboteq.hpp"
#include "roboteq/channel.hpp"
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <msgs/StringStamped.h>
#include <msgs/IntStamped.h>
#include <cstdarg>

class hbl1650 : public RoboTeQ
{
private:
	Channel controller;

	ros::NodeHandle 		local_node_handler;
	ros::NodeHandle 		global_node_handler;

	bool 					deadman_pressed,
							cmd_vel_publishing,
							initialised,
							controller_responding;

	int 					velocity;

	std_msgs::Bool 			prev_joy;

	ros::Time 				last_deadman_received;
	ros::Time 				last_twist_received;

public:
	bool 					closed_loop_operation,
							emergency_stop;

	int						p_gain,
							i_gain,
							d_gain,
							max_rpm,
							anti_windup_percent,
							max_acceleration,
							max_deceleration,
							velocity_max;

	double 					mps_to_rpm;

	ros::Subscriber 		serial_sub,
							command_relay_sub,
							cmd_vel_sub,
							deadman_sub;

	ros::Duration 			max_time_diff;

							hbl1650();
	void					spin(void);
	void 					initController(void);

	void					onCmdVel(const geometry_msgs::TwistStamped::ConstPtr&);

	void					onDeadman(const std_msgs::Bool::ConstPtr&);
	void					onTimer(const ros::TimerEvent&);
	void 					onSerial(const msgs::serial::ConstPtr& msg){serialCallback(msg);}
	void 					onCommand(const msgs::serial::ConstPtr& msg);

	void 					setChannels(bool ch){two_channel = ch;}
	void					setSerialPub(ros::Publisher pub){serial_publisher = pub;}
	void					setStatusPub(ros::Publisher pub){status_publisher = pub;}
	void					setTemperaturePub(ros::Publisher pub){temperature_publisher = pub;}
	void					setPowerPub(ros::Publisher pub){controller.publisher.power = pub;}
	void					setEncoderPub(ros::Publisher pub){controller.publisher.hall = pub;}
	int						subscribers(){return serial_publisher.getNumSubscribers();}

};

#endif /* HBL1650_HPP_ */
