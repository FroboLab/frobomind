/*
 * RoboTeQ.hpp
 *
 *  Created on: Jan 26, 2013
 *      Author: leonbondelarsen
 */

#ifndef ROBOTEQ_HPP_
#define ROBOTEQ_HPP_

#include <ros/ros.h>
#include <iostream>
#include <msgs/serial.h>
#include <msgs/IntStamped.h>
#include <msgs/StringStamped.h>

class RoboTeQ
{
public:
	bool					online,
							two_channel,
							hall_cb1_initialised,
							hall_cb2_initialised,
							power_cb1_initialised,
							power_cb2_initialised,
							temperature_cb1_initialised,
							temperature_cb2_initialised;

	unsigned int			ff,fs;

	int						cb1,cb2,
							cbr1,cbr2,
							f1,f2,
							a1,a2,
							ba1,ba2,
							bs1,bs2,
							bsr1,bsr2,
							e1,e2,
							p1,p2,
							t1,t2,t3,
							v1,v2,v3;

	msgs::serial 			serial_out;

	msgs::StringStamped		status_out;

	ros::Time 				last_serial_msg;

	ros::Publisher 			serial_publisher, status_publisher, temperature_publisher;

	std::string 			statusFlagsToString(unsigned int);
	std::string 			faultFlagsToString(unsigned int);

	void 					(*hall_ch1_callback)(ros::Time time , int value);
	void 					(*hall_ch2_callback)(ros::Time time , int value);
	void 					(*power_ch1_callback)(ros::Time time , int value);
	void 					(*power_ch2_callback)(ros::Time time , int value);

							RoboTeQ();

	void 					serialCallback(const msgs::serial::ConstPtr&);

	void					registerHallCb(int channel, void (*callback)(ros::Time time , int value));
	void					registerPowerCb(int channel, void (*callback)(ros::Time time , int value));

	void 					hall_feedback(ros::Time time, int fb1, int fb2){hall_ch1_callback(time,fb1);hall_ch2_callback(time,fb2);}
	void					hall_feedback(ros::Time time, int fb1){hall_ch1_callback(time,fb1);}
	void 					power_feedback(ros::Time time, int fb1, int fb2){power_ch1_callback(time,fb1);power_ch2_callback(time,fb2);}
	void					power_feedback(ros::Time time, int fb1){power_ch1_callback(time,fb1);}

	void					transmit(int, std::string , ...);
};

#endif /* ROBOTEQ_HPP_ */
