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
 # 2013-06-02 Leon: Implemented regulator
 #
 #
 ****************************************************************************/
#ifndef CHANNEL_HPP_
#define CHANNEL_HPP_

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <msgs/StringStamped.h>
#include <msgs/IntStamped.h>
#include <msgs/PropulsionModuleFeedback.h>
#include "roboteq_hbl2350/roboteq.hpp"
#include "roboteq_hbl2350/regulator.hpp"
#include "filter/IRR.h"
#include "filter/sliding_window.hpp"

class BaseCB
{
/*
 * Abstract class overloading the function call operator with a pure virtual function
 * */
public:
	virtual void operator()(std::string)=0;
};


template<class ClassT>
class CallbackHandler : public BaseCB
{
/*
 * Templated placeholder class for handling callbacks to member functions
 * */
public:
	typedef void(ClassT::* FuncT)(std::string);
	FuncT _fn;
	ClassT* _c;

	CallbackHandler(ClassT* c, FuncT fn):_fn(fn),_c(c){}
	void operator()(std::string str)
	{
		return (_c->*_fn )(str);
	}
};

class Channel
{
/*
 * Class implementing the concept of a motCircular_queue::or controller channel
 * */
public:
	// Convenience structs for holding related variables
	struct
	{
		msgs::IntStamped hall, power, temperature;
		msgs::StringStamped status;
		msgs::PropulsionModuleFeedback feedback;
	} message;

	struct
	{
		ros::Publisher power, hall, temperature, feedback, status;
	} publisher;

	struct
	{
		ros::Time last_twist_received, last_deadman_received, last_regulation;
	} time_stamp;

	ros::Subscriber cmd_vel_sub;

	int	ch, last_hall, anti_windup_percent, max_acceleration, max_deceleration, roboteq_max, hall_value,down_time,max_rpm, current_thrust;
	double i_max, max_output, current_setpoint, velocity,mps_to_rpm,p_gain, i_gain, d_gain, ticks_to_meter, max_velocity_mps, mps_to_thrust;

	IRR velocity_filter;
	SlidingWindowFilter feedback_filter;
	Regulator regulator;
	BaseCB* transmit_cb;
	BaseCB* init_cb;


	Channel();

	// Callbacks to RoboTeQ derived class
	void transmit(std::string str){(*transmit_cb)(str);}
	void initController(std::string str){(*init_cb)(str);}

	// Callbacks from RoboTeQ derived class
	void onHallFeedback(ros::Time time, int feedback);
	void onPowerFeedback(ros::Time time, int feedback);
	void onTemperatureFeedback(ros::Time time, int feedback);

	// Subscriber callbacks
	void onCmdVel(const geometry_msgs::TwistStamped::ConstPtr&);
	void onDeadman(const std_msgs::Bool::ConstPtr&);
	void onTimer(const ros::TimerEvent&, RoboTeQ::status_t&);

	// Mutator method for setting up publisher
	void setStatusPub(ros::Publisher pub){publisher.status = pub;}
	void setPropulsionFeedbackPub(ros::Publisher pub){publisher.feedback = pub;}
	void setVelPub(ros::Publisher pub){publisher.feedback = pub;}

};

#endif /* CHANNEL_HPP_ */
