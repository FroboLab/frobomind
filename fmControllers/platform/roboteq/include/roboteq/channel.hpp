#ifndef CHANNEL_HPP_
#define CHANNEL_HPP_

#define TIME_BETWEEN_COMMANDS 0.2

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <msgs/StringStamped.h>
#include <msgs/IntStamped.h>
//#include <cstdarg>
#include "roboteq/roboteq.hpp"
#include "roboteq/regulator.hpp"

class Channel
{
public:
	struct
	{
		msgs::IntStamped hall, power, temperature;
		msgs::StringStamped status;
	} message;

	struct
	{
		ros::Publisher power, hall, temperature;
	} publisher;

//	bool deadman_pressed,cmd_vel_publishing,initialised,controller_responding, emergency_stop;
//	int	max_rpm, anti_windup_percent, max_acceleration, max_deceleration, velocity_max;
//	double velocity,mps_to_rpm,p_gain, i_gain, d_gain;
//	ros::Time last_twist_received, last_deadman_received;
//	ros::Duration max_time_diff;
//	ros::Subscriber cmd_vel_sub;
//	Regulator regulator;

	Channel();
	void onHallFeedback(ros::Time time, int feedback);
	void onPowerFeedback(ros::Time time, int feedback);
	void onTemperatureFeedback(ros::Time time, int feedback);
//	bool initController(void);
//	void CmdVel(const geometry_msgs::TwistStamped::ConstPtr&);
//	void Deadman(const std_msgs::Bool::ConstPtr&);
//	void Timer(const ros::TimerEvent&);
//	void Command(const msgs::serial::ConstPtr& msg);
//	void Feedback(const msgs::serial::ConstPtr& msg);
};

#endif /* CHANNEL_HPP_ */
