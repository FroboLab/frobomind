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

//Abstract class overloading the function call operator
class BaseCB
{
public:
	virtual void operator()(std::string)=0;
};

// Callback placeholder
template<class ClassT>
class CallbackHandler : public BaseCB
{
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

	struct status_t
	{
		bool online, deadman_pressed, cmd_vel_publishing, initialised, responding, emergency_stop;
	};

	int	ch, max_rpm, anti_windup_percent, max_acceleration, max_deceleration, roboteq_max, hall_value;
	double velocity,mps_to_rpm,p_gain, i_gain, d_gain, ticks_to_mps, max_velocity_mps;
	ros::Time last_twist_received, last_deadman_received, last_regulation;
	ros::Duration max_time_diff;
	ros::Subscriber cmd_vel_sub;
	ros::Publisher status_publisher;
	msgs::StringStamped	status_out;
	Regulator regulator;

	BaseCB* transmit_cb;
	BaseCB* init_cb;
	void transmit(std::string str){(*transmit_cb)(str);}
	void initController(std::string str){(*init_cb)(str);}

	Channel();
	void onHallFeedback(ros::Time time, int feedback);
	void onPowerFeedback(ros::Time time, int feedback);
	void onTemperatureFeedback(ros::Time time, int feedback);

	//bool initController(void);

	void onCmdVel(const geometry_msgs::TwistStamped::ConstPtr&);
	void onDeadman(const std_msgs::Bool::ConstPtr&);
	void onTimer(const ros::TimerEvent&, status_t);
	void setStatusPub(ros::Publisher pub){status_publisher = pub;}
//	void Command(const msgs::serial::ConstPtr& msg);
//	void Feedback(const msgs::serial::ConstPtr& msg);
};

#endif /* CHANNEL_HPP_ */
