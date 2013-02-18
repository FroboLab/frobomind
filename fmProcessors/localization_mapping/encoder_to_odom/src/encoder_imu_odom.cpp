/*
 * simple_odom.cpp
 *
 *  Created on: Apr 25, 2012
 *      Author: morl
 */
#include <ros/ros.h>
#include <ros/console.h>

#include <msgs/encoder.h>
#include <nav_msgs/Odometry.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <string>

using namespace std;

class SimpleOdom
{
public:
	SimpleOdom(double conv_l,double conv_r,double vl,double max_diff)
	{
		c_r = conv_r;
		c_l = conv_l;
		v_l = vl;
		this->max_diff = max_diff;

		delta_l = delta_r = 0;

		l_updated = r_updated = l_ready = r_ready = i_updated  =false;

		prev_imu_attitude_yaw = -1000;
		
	}

	~SimpleOdom()
	{

	}

	void processImu(const sensor_msgs::Imu::ConstPtr& msg)
	{
		double q0 = msg->orientation.x;
		double q1 = msg->orientation.y;
		double q2 = msg->orientation.z;
		double q3 = msg->orientation.w;
		imu_attitude_yaw = atan2 (2*(q0*q1+q3*q2),(pow(q3,2)-pow(q2,2)-pow(q1,2)+pow(q0,2)));
		i_updated=true;
	}


	void processLeftEncoder(const msgs::encoder::ConstPtr& msg)
	{
		if(!l_ready)
		{
			prev_l = *msg;
			l_ready = true;
		}
		else
		{
			delta_l += (msg->encoderticks - prev_l.encoderticks) * c_l;
			l_up_time = ros::Time::now();
			l_updated = true;
			prev_l = *msg;
		}

	}

	void processRightEncoder(const msgs::encoder::ConstPtr& msg)
	{
		if(!r_ready)
		{
			prev_r = *msg;
			r_ready = true;
		}
		else
		{
			delta_r += (msg->encoderticks - prev_r.encoderticks) * c_r;
			r_up_time = ros::Time::now();
			r_updated = true;
			prev_r = *msg;
		}

	}

	void publishOdometry(const ros::TimerEvent& e)
	{
		
		if(l_updated && r_updated && i_updated )
		{
			if(prev_imu_attitude_yaw!=-1000){


				// check update times
				if((l_up_time - r_up_time).toSec() > max_diff)
				{
					ROS_WARN("Encoder stamp (left - right) differs %.4f",(l_up_time - r_up_time).toSec());
				}

				r_updated = l_updated = false;

				// calculate distance vector
				double dx = (delta_l + delta_r)/2;
				ROS_INFO("dx is %f",dx);

				// calculate change in orientation based on distance between wheels
				

				double dtheta = imu_attitude_yaw - prev_imu_attitude_yaw;
				if (dtheta < -M_PI)  
				 	dtheta += 2*M_PI;
				else if(dtheta > M_PI)
					dtheta -= 2*M_PI;



				geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(imu_attitude_yaw);

				odom_trans.header.stamp = ros::Time::now();
				odom_trans.header.frame_id = odom_frame;
				odom_trans.child_frame_id = base_frame;
				odom_trans.transform.translation.x += cos(imu_attitude_yaw)*dx;
				odom_trans.transform.translation.y += sin(imu_attitude_yaw)*dx;
				odom_trans.transform.translation.z = 0.0;
				odom_trans.transform.rotation = odom_quat;

				odom_broadcaster.sendTransform(odom_trans);

				odom.header.stamp = ros::Time::now();
				odom.header.frame_id = "odom_combined";

				odom.pose.pose.position.x = odom_trans.transform.translation.x;
				odom.pose.pose.position.y = odom_trans.transform.translation.y;
				odom.pose.pose.orientation = odom_quat;

				odom.twist.twist.linear.x  = dx;
				odom.twist.twist.angular.z = dtheta;

				odom_pub.publish(odom);

				delta_l = delta_r = 0;
			}

		}
		prev_imu_attitude_yaw = imu_attitude_yaw;

	}

	ros::Publisher odom_pub;
	tf::TransformBroadcaster odom_broadcaster;
	std::string base_frame,odom_frame;

private:
	double c_l,c_r,v_l;
	double delta_l,delta_r;
	bool l_updated,r_updated,i_updated;

	double imu_attitude_yaw;
	double prev_imu_attitude_yaw;

	ros::Time l_up_time,r_up_time;
	bool l_ready,r_ready;
	double max_diff;

	msgs::encoder prev_l,prev_r;
	nav_msgs::Odometry odom;

	geometry_msgs::TransformStamped odom_trans;


};

int main(int argc, char** argv) {

	ros::init(argc, argv, "odometry_publisher");

	ros::NodeHandle nh("~");
	ros::NodeHandle n;

	string publish_topic;
	string subscribe_enc_l;
	string subscribe_enc_r;
	string subscribe_imu;


	double v_l,r_t_m,l_t_m,max_diff;
	ros::Subscriber s1,s2,s3;


	nh.param<string>("publisher_topic", publish_topic, "/fmExtractors/odom");
	nh.param<string>("subscribe_imu",subscribe_imu, "/fmSensors/IMU");
	nh.param<string>("enc_r_subscriber_topic", subscribe_enc_r,
			"/fmSensors/encoder_right");
	nh.param<string>("enc_l_subscriber_topic", subscribe_enc_l,
			"/fmSensors/encoder_left");

	nh.param<double>("conv_ticks_to_meter_left", l_t_m, 0.001335);
	nh.param<double>("conv_ticks_to_meter_right", r_t_m, 0.001335);
	nh.param<double>("max_time_diff", max_diff,1);



	nh.param<double>("distance_between_wheels_in_meter", v_l, 1.03);

	SimpleOdom p(l_t_m,r_t_m,v_l,max_diff);

	nh.param<std::string>("vehicle_frame",p.base_frame,"base_footprint");
	nh.param<std::string>("odom_estimate_frame",p.odom_frame,"odom_combined");

	s1 = nh.subscribe(subscribe_enc_l,15,&SimpleOdom::processLeftEncoder,&p);
	s2 = nh.subscribe(subscribe_enc_r,15,&SimpleOdom::processRightEncoder,&p);

	s3 = nh.subscribe(subscribe_imu,15,&SimpleOdom::processImu,&p);

	p.odom_pub = n.advertise<nav_msgs::Odometry>(publish_topic.c_str(), 25);
	ros::Timer t;
	t = nh.createTimer(ros::Duration(1/30.0), &SimpleOdom::publishOdometry,&p);

	ros::spin();

	return 0;
}




