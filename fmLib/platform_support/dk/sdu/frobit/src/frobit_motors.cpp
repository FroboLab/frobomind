#include "ros/ros.h"
#include <string.h>
#include <sstream>
#include "boost/lexical_cast.hpp"
#include "msgs/nmea.h"
#include "msgs/encoder.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/TwistStamped.h>

class FroboMotor
{
private:

	bool active;
	geometry_msgs::TwistStamped msg_left,msg_right;
	msgs::nmea duty_message;
	msgs::encoder encoder_message;

public:
	double vel_to_motor_const;
	double timeout,max_velocity;
	bool deadman;
	ros::Publisher nmea_pub, encoder_pub_left, encoder_pub_right;
	ros::Time last_deadman_received;

	FroboMotor()
	{

	};

	~FroboMotor()
	{

	};

	void on_vel_msg_left(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		msg_left = *msg;
	}

	void on_vel_msg_right(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		msg_right = *msg;
	}

	void on_deadman(const std_msgs::Bool::ConstPtr& msg)
	{
		deadman = msg->data;
		last_deadman_received = ros::Time::now();
	}

	void on_encoder(const msgs::nmea::ConstPtr& msg)
	{
		encoder_message.header.stamp = msg->header.stamp;
		encoder_message.encoderticks = boost::lexical_cast<int>(msg->data.at(1));
		encoder_pub_left.publish(encoder_message);

		encoder_message.encoderticks = boost::lexical_cast<int>(msg->data.at(2));
		encoder_pub_right.publish(encoder_message);
	}

	void on_timer(const ros::TimerEvent& e)
	{
		double left_vel = 0;
		double right_vel = 0;
		active = true;
		if((ros::Time::now() - msg_left.header.stamp).toSec() > timeout)
		{
			ROS_WARN_THROTTLE(1,"Time for left cmd_vel is out of date");
			active = false;
		}

		if((ros::Time::now() - msg_right.header.stamp).toSec() > timeout)
		{
			ROS_WARN_THROTTLE(1,"Time for right cmd_vel is out of date");
			active = false;
		}

		if( ros::Time::now() > last_deadman_received + ros::Duration(0.5) )
		{
			deadman = false;
		}

		if(active && deadman)
		{
			left_vel = msg_left.twist.linear.x * 100;
			right_vel = msg_right.twist.linear.x * 100;

			//correct high velocities
			if ( left_vel > max_velocity )
				left_vel = max_velocity;
			else if ( left_vel < - max_velocity )
				left_vel = - max_velocity;

			if ( right_vel > max_velocity )
				right_vel = max_velocity;
			else if ( right_vel < - max_velocity )
				right_vel = - max_velocity;

			left_vel *= vel_to_motor_const;
			right_vel *= vel_to_motor_const;
		}
		else
		{
			left_vel = 0;
			right_vel = 0;
		}

		//build message
		duty_message.header.stamp = ros::Time::now();
		duty_message.type = "PFBCT";
		duty_message.data.clear();
		duty_message.data.push_back( boost::lexical_cast<std::string>( (int)right_vel ) );
		duty_message.data.push_back( boost::lexical_cast<std::string>( (int)left_vel ) );

		//publish message
		nmea_pub.publish(duty_message);
	}

};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "frobit_motorcontroller");

	ros::NodeHandle 	nh, n("~");

	std::string 		vel_l,
						vel_r,
						deadman,
						nmea_from_frobit,
						encoder_left_topic,
						encoder_right_topic,
						publish_topic_id;

	double 				wheel_diameter,
						ticks_pr_round,
						ms_in_between,
						interval;

	ros::Subscriber 	s1,
						s2,
						s3,
						s4;

	FroboMotor frobit_motor;

	frobit_motor.deadman = false;

	n.param<std::string>("frobit_nmea_sub", nmea_from_frobit, "/fmData/nmea_from_frobit");
	n.param<std::string>("deadman", deadman, "/fmSignals/deadman");
	n.param<std::string>("frobit_velocity_sub_left", vel_l, "/fmDecision/twist");
	n.param<std::string>("frobit_velocity_sub_right", vel_r, "/fmDecision/twist");
	n.param<std::string>("frobit_nmea_pub", publish_topic_id, "/fmActuators/duty");
	n.param<std::string>("encoder_left_pub", encoder_left_topic, "/fmInformation/encoder_left");
	n.param<std::string>("encoder_right_pub", encoder_right_topic, "/fmInformation/encoder_right");

	n.param<double>("max_velocity", frobit_motor.max_velocity,255);
	n.param<double>("wheel_diameter", wheel_diameter, 0.10);
	n.param<double>("ticks_pr_round", ticks_pr_round, 360);
	n.param<double>("ms_in_between", ms_in_between, 100);
	n.param<double>("vel_publish_interval",interval,0.05);
	n.param<double>("vel_timeout",frobit_motor.timeout,1);

	/* m/s -> ticks/entry * 1 m/s = 1/(pi*wheel_diameter) rps = ticks_pr_round/(pi*wheel_diameter) ticks/sec = */
	frobit_motor.vel_to_motor_const = ( ticks_pr_round / ( 3.14 * wheel_diameter ) ) * ( 1 / ms_in_between );

	s1 = nh.subscribe<geometry_msgs::TwistStamped>(vel_l,2,&FroboMotor::on_vel_msg_left,&frobit_motor);
	s2 = nh.subscribe<geometry_msgs::TwistStamped>(vel_r,2,&FroboMotor::on_vel_msg_right,&frobit_motor);
	s3 = nh.subscribe<std_msgs::Bool>(deadman,2,&FroboMotor::on_deadman,&frobit_motor);
	s4 = nh.subscribe<msgs::nmea>(nmea_from_frobit,2,&FroboMotor::on_encoder,&frobit_motor);


	frobit_motor.nmea_pub = nh.advertise<msgs::nmea>(publish_topic_id, 1);
	frobit_motor.encoder_pub_left = nh.advertise<msgs::encoder>(encoder_left_topic, 1);
	frobit_motor.encoder_pub_right = nh.advertise<msgs::encoder>(encoder_right_topic, 1);

	ros::Timer t1= nh.createTimer(ros::Duration(interval),&FroboMotor::on_timer,&frobit_motor);

	ros::spin();
	return 0;
}
