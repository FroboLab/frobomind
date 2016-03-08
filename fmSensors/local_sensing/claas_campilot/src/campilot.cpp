/****************************************************************************
# FroboMind claas_eye_drive_node
# Copyright (c) 2011-2013 Morten Larsen mortenlarsens@gmail.com
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#	* Redistributions of source code must retain the above copyright
#  	notice, this list of conditions and the following disclaimer.
#	* Redistributions in binary form must reproduce the above copyright
#  	notice, this list of conditions and the following disclaimer in the
#  	documentation and/or other materials provided with the distribution.
#       * Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
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
*****************************************************************************

2016-03-08 Kjeld Jensen kjen@mmmi.sdu.dk
  Updated code together with CLAAS, added support for sending Machine info
  messages to the eye_drive

****************************************************************************/
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <msgs/can.h>
#include <msgs/claas_campilot.h>

typedef enum
{
	STOP = 0x40,
	INIT1 = 0x20,
	INIT2 = 0x21,
	INIT3 = 0x22,
	NO_ENTRY = 0x00

}CMDControl_t;

class CameraConfig
{
public:
	CameraConfig()
	{
		debug_setting = 0x1C;
		program = 0x03;
		height_cm = 160;
		angle_deg = 45;
		threshold = 0x80;
		target_width_cm = 0x0F;
		target_height_cm = 0x0C;
		target_distance_cm = 0x4B;
		target_minor_distance_cm = 0x00;
		number_of_rows = 0x01;
		rows_between_wheels = 0x03;
	}

	void print_config()
	{

	}

	void to_init_message(uint8_t* msg)
	{
		msg[0] = INIT1;
		msg[1] = program;
		msg[2] = height_cm >> 8;
		msg[3] = height_cm;
		msg[4] = angle_deg;
		msg[5] = threshold;
		msg[6] = debug_setting;
		msg[7] = NO_ENTRY;

		msg[8] 	= INIT2;
		msg[9] 	= target_width_cm;
		msg[10] = target_height_cm >> 8;
		msg[11] = target_height_cm;
		msg[12] = target_distance_cm >> 8;
		msg[13] = target_distance_cm;
		msg[14] = target_minor_distance_cm;
		msg[15] = number_of_rows;

		msg[16] = INIT3;
		msg[17] = rows_between_wheels;
		msg[18] = msg[19] = msg[20] = msg[21] = msg[22] = msg[23] = NO_ENTRY;
	}

	int program;
	int height_cm;
	int angle_deg;
	int threshold;
	int target_width_cm;
	int target_height_cm;
	int target_distance_cm;
	int target_minor_distance_cm;
	int number_of_rows;
	int rows_between_wheels;
	int debug_setting;
};

class CamPilot
{
public:
	CamPilot(CameraConfig config)
	{
		this->config = config;
		timeout = 1;
		communication_timeout = 5;
		is_initialised = false;
		first_time_init = false;
		quality = heading = offset = 0.0;
		last_update = ros::Time::now();
		user_angle = 0;
		user_offset = 0;
		machine_auto_mode = false;
		machine_velocity = false;
	}

	~CamPilot()
	{

	}

	void initCamera()
	{
		uint8_t buf[24];
		config.to_init_message(buf);

		transmitStopMsg();
		usleep (100000); /* 100ms delay required by eye_drive */
		for(int i=0;i<3;i++)
		{
			transmitInitMsg(&(buf[i*8]));
			usleep (100000); /* 100ms delay required by eye_drive */
		}
	}

	void processCanRxEvent(const msgs::can::ConstPtr& msg)
	{
		double temp_val;
		if(msg->id == (uint32_t)(0x142000C8))
		{
			//ROS_INFO("Got data");
			quality = msg->data[1];

			temp_val = (msg->data[2]<<8) + msg->data[3];
			if (temp_val > 0x8000) /* Convert Two's complement: */
			{
			        temp_val =  temp_val - 0x10000;
			}
			temp_val =  temp_val/10;
			offset = temp_val;

			temp_val = (msg->data[4]<<8) + msg->data[5];
			if (temp_val > 0x8000) /* Convert Two's complement: */
			{
			        temp_val =  temp_val - 0x10000;
			}
			temp_val =  temp_val/100;
			heading = temp_val;
			debug = (((msg->data[6]<<8) + msg->data[7])/100);

			if((msg->header.stamp - last_update).toSec() > timeout)
			{
				ROS_WARN_THROTTLE(1,"Timeout value exceeded");
			}

			last_update = msg->header.stamp;
		}
		if(msg->id == 201326776) /* 0xC0000B8 */
		{
			// ROS_INFO("Got heartbeat");
			if (first_time_init == false)
			{	
				initCamera();
				ROS_INFO("Initializing");
				first_time_init = true;
			}
		}
	}

	void processMachineAutoModeEvent(const std_msgs::Bool::ConstPtr& msg)
	{
		machine_auto_mode = msg->data;
	}

	void processMachineVelocityEvent(const std_msgs::Int32::ConstPtr& msg)
	{
		machine_velocity = msg->data;
	}

	void processTimerEvent(const ros::TimerEvent& e)
	{
		if(!is_initialised)
		{
			if(can_tx_pub.getNumSubscribers() == 0)
			{
				ROS_INFO_THROTTLE(1,"Waiting for can node to subscribe");
			}
			else
			{
				initCamera();
				is_initialised = true;
				last_update = ros::Time::now();
			}
		}
		else
		{
			ros::Time t = ros::Time::now();
			if( (t - last_update).toSec() > communication_timeout)
			{
				is_initialised = false;
				ROS_WARN_THROTTLE(1,"Lost Connection to eye drive, retrying");
				quality = heading = offset = 0;
			}
			cam_tx_msg.header.stamp = ros::Time::now();
			cam_tx_msg.header.frame_id = frame_id;
			cam_tx_msg.quality = quality;
			cam_tx_msg.heading = heading;
			cam_tx_msg.offset = offset;


			cam_row_pub.publish(cam_tx_msg);
		}


	}

	ros::Publisher can_tx_pub;
	ros::Subscriber can_rx_sub;
	ros::Subscriber machine_auto_mode_sub;
	ros::Subscriber machine_velocity_sub;
	ros::Publisher cam_row_pub;

	std::string frame_id;
private:

	void transmitMachineInfoMsg(void)
	{
		can_tx_msg.header.stamp = ros::Time::now();
		can_tx_msg.flags = 0x04; // EFF is indicated in flags (atleast when using can4linux)
		can_tx_msg.id = 0x1425003C;
		can_tx_msg.length = 8;
		can_tx_msg.data[0] = 0; /* indicate 100 ms message interval (int msb) */
		can_tx_msg.data[1] = 100; /* indicate 100 ms message interval (int lsb) */
		can_tx_msg.data[2] = machine_auto_mode; /* 0=manual, 1=auto */
		can_tx_msg.data[3] = machine_velocity >> 8; /* machine velocity [mm/s] (int msb) */
		can_tx_msg.data[4] = machine_velocity & 0xff; /* machine velocity [mm/s] (int lsb) */
		can_tx_msg.data[5] = user_angle >> 8; /* not used in current setup (int msb) */
		can_tx_msg.data[6] = user_angle & 0xff; /* not used in current setup (int msb) */
		can_tx_msg.data[7] = user_offset; /* not used in current setup */
		can_tx_pub.publish(can_tx_msg);
	}

	void transmitStopMsg(void)
	{
		can_tx_msg.header.stamp = ros::Time::now();
		can_tx_msg.flags = 0x04; // EFF is indicated in flags (atleast when using can4linux)
		can_tx_msg.id = 0x1424003C;
		can_tx_msg.length = 8;
		can_tx_msg.data[0] = STOP;
		for(int i=1;i<8;i++)
		{
			can_tx_msg.data[i] = 0;
		}

		can_tx_pub.publish(can_tx_msg);
	}


	void transmitInitMsg(uint8_t buf[8])
	{
		can_tx_msg.header.stamp = ros::Time::now();
		can_tx_msg.flags = 0x04; // EFF is indicated in flags (atleast when using can4linux)
		can_tx_msg.id = 0x1424003C;
		can_tx_msg.length = 8;
		for(int i=0;i<8;i++)
		{
			can_tx_msg.data[i] =buf[i];
		}

		can_tx_pub.publish(can_tx_msg);
	}

	CameraConfig config;

	msgs::can can_tx_msg;
	msgs::claas_campilot cam_tx_msg;

	ros::Time last_update;

	int quality;
	double heading;
	double offset;
	int debug;

	bool machine_auto_mode;
	unsigned int machine_velocity;
	unsigned int user_angle;
	unsigned char user_offset;

	bool first_time_init;
	bool is_initialised;

	double timeout;
	double communication_timeout;
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "claas_campilot_node");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	std::string can_rx_topic,can_tx_topic,machine_auto_mode_sub_topic, machine_velocity_sub_topic, row_topic,frame_id;
	double publish_rate;

	CameraConfig conf;

	ros::Timer t;

	n.param<std::string>("can_from_device_sub",can_rx_topic,"/fmSignal/can_from_campilot");
	n.param<std::string>("can_to_device_pub",can_tx_topic,"/fmSignal/can_to_campilot");
	n.param<std::string>("machine_auto_mode_sub",machine_auto_mode_sub_topic,"/fmPlan/cam_machine_automode");
	n.param<std::string>("machine_velocity_sub",machine_velocity_sub_topic,"/fmKnowledge/cam_machine_velocity");
	n.param<std::string>("cam_rows_pub",row_topic,"/fmKnowledge/cam_rows");


	n.param<int>("cam_program",conf.program,0x03);
	n.param<int>("cam_height",conf.height_cm,160);
	n.param<int>("cam_angle",conf.angle_deg,45);
	n.param<int>("cam_threshold",conf.threshold,0x80);
	n.param<int>("cam_target_width",conf.target_width_cm,15);
	n.param<int>("cam_target_height",conf.target_height_cm,12);
	n.param<int>("cam_target_distance",conf.target_distance_cm,75);
	n.param<int>("cam_minor_distance",conf.target_minor_distance_cm,75);
	n.param<int>("cam_number_of_rows",conf.number_of_rows,1);
	n.param<int>("cam_rows_between_wheels",conf.rows_between_wheels,0x03);

	n.param<double>("publish_rate",publish_rate,25.0);
	n.param<std::string>("frame_id",frame_id,"campilot_link");


	CamPilot camera(conf);

	camera.frame_id = frame_id;
	camera.can_rx_sub  = nh.subscribe<msgs::can> (can_rx_topic.c_str(),10,&CamPilot::processCanRxEvent,&camera);
	camera.can_tx_pub  = nh.advertise<msgs::can> (can_tx_topic.c_str(),10);
	camera.machine_auto_mode_sub = nh.subscribe<std_msgs::Bool> (machine_auto_mode_sub_topic.c_str(),10,&CamPilot::processMachineAutoModeEvent,&camera);
	camera.machine_velocity_sub = nh.subscribe<std_msgs::Int32> (machine_velocity_sub_topic.c_str(),10,&CamPilot::processMachineVelocityEvent,&camera);
	camera.cam_row_pub = nh.advertise<msgs::claas_campilot> (row_topic.c_str(),1);

	t= nh.createTimer(ros::Duration(1.0/publish_rate),&CamPilot::processTimerEvent,&camera);


	ros::spin();

}
