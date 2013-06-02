#ifndef HBL2350_HPP_
#define HBL2350_HPP_

#define TIME_BETWEEN_COMMANDS 0.2

#include <ros/ros.h>
#include "roboteq/roboteq.hpp"
#include "roboteq/channel.hpp"
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <msgs/StringStamped.h>
#include <msgs/IntStamped.h>
#include <msgs/serial.h>
#include <cstdarg>

class hbl2350 : public RoboTeQ
{
private:
	Channel 				ch1,ch2;

	ros::NodeHandle 		local_node_handler;
	ros::NodeHandle 		global_node_handler;

	//Channel::status_t 		status;
//
//	int 					velocity_ch1,
//							velocity_ch2;
//
//	std_msgs::Bool 			prev_joy;
//
//	ros::Time 				last_deadman_received;
//	ros::Time 				last_twist_received_ch1;
//	ros::Time 				last_twist_received_ch2;

public:
	bool 					closed_loop_operation;
							//emergency_stop;

//	int						p_gain_ch1,
//							i_gain_ch1,
//							d_gain_ch1,
//							p_gain_ch2,
//							i_gain_ch2,
//							d_gain_ch2,
//							max_rpm,
//							anti_windup_percent,
//							max_acceleration,
//							max_deceleration,
//							velocity_max;

	double 					mps_to_rpm;

	ros::Subscriber 		serial_sub,
							command_relay_sub,
//							cmd_vel_ch1_sub,
//							cmd_vel_ch2_sub,
							deadman_sub;

	ros::Duration 			max_time_diff;

							hbl2350();
	void					spin(void);
	void 					initController(std::string);
	void					updateStatus(void);

	void					onDeadman(const std_msgs::Bool::ConstPtr& msg){ch1.onDeadman(msg); ch2.onDeadman(msg);}
	void					onTimer(const ros::TimerEvent& event){updateStatus(); ch1.onTimer(event,status);ch2.onTimer(event,status);}
	void 					onSerial(const msgs::serial::ConstPtr& msg){serialCallback(msg);}
	//void 					onCommand(const msgs::serial::ConstPtr& msg);

	void					setSerialPub(ros::Publisher pub){serial_publisher = pub;}
	void					setStatusPub(ros::Publisher pub){status_publisher = pub;}
	void					setTemperaturePub(ros::Publisher pub){temperature_publisher = pub;}
	void					setPowerCh1Pub(ros::Publisher pub){ch1.publisher.power = pub;}
	void					setPowerCh2Pub(ros::Publisher pub){ch2.publisher.power = pub;}
	void					setEncoderCh1Pub(ros::Publisher pub){ch1.publisher.hall = pub;}
	void					setEncoderCh2Pub(ros::Publisher pub){ch2.publisher.hall = pub;}

	int						subscribers(){return serial_publisher.getNumSubscribers();}



};

#endif /* HBL2350_HPP_ */
