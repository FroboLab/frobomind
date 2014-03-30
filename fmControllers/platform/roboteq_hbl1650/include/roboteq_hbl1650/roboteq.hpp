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
 ****************************************************************************
 # This class implements the low level communication to the RoboTeQ controllers.
 # 
 ****************************************************************************/
#ifndef ROBOTEQ_HPP_
#define ROBOTEQ_HPP_

#include <ros/ros.h>
#include <iostream>
#include <msgs/serial.h>
#include <msgs/IntStamped.h>
#include <msgs/StringStamped.h>
#include <msgs/PropulsionModuleStatus.h>

class RoboTeQ
{
public:
	// Convenience struct for holding status booleans
	struct status_t
	{
		bool online, 		// True when controller is online
		deadman_pressed, 	// True when deadman button is pressed
		cmd_vel_publishing, 	// True if cmd_vel message has been received within time limit
		initialised, 		// True if controller is initialised
		responding, 		// True is serial message has been received within time limit
		emergency_stop;		// True if emgency stop is activated !
	} status;

	bool two_channel;		// True if controller has two channel mode
	unsigned int ff,fs;		// Variables for holding status flags
	int cb1,cb2,			// Variables for holding absolute hall values
	a1,a2,				// Variables for holding motor current readings
	ba1,ba2,			// Variables for holding battery current readings
	p1,p2,				// Variables for holding power readings
	t1,t2,t3,			// Variables for holding temperature readings
	v1,v2,v3;			// Variables for holding voltage readings

	msgs::serial serial_out;
	msgs::StringStamped status_out;
	msgs::PropulsionModuleStatus propulsion_module_status_message;
	ros::Time last_serial_msg;
	ros::Publisher serial_publisher, status_publisher, temperature_publisher, propulsion_module_status_publisher;

	RoboTeQ();

	// Conversion methods
	std::string statusFlagsToString(unsigned int);
	std::string faultFlagsToString(unsigned int);

	// Method for handling incoming serial communication
	void serialCallback(const msgs::serial::ConstPtr&);

	// Methods for handling feedback from the controller. Implemented as virtual to
	// allow inheriting class to override. Notice the empty implementation.
	virtual void hall_feedback(ros::Time time, int fb1, int fb2){}
	virtual void hall_feedback(ros::Time time, int fb1){}
	virtual void power_feedback(ros::Time time, int fb1, int fb2){}
	virtual void power_feedback(ros::Time time, int fb1){}
	virtual void initController( std::string ){}

	// Methods for transmitting commands to the controller
	void transmit(int, std::string , ...);
	void transmit(std::string);
};

#endif /* ROBOTEQ_HPP_ */
