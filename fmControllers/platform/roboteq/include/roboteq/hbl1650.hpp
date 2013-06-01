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
