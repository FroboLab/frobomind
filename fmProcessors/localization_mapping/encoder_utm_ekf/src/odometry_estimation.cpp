/*
 * odometry_estimation.cpp
 *
 *  Created on: Apr 22, 2012
 *      Author: morl
 */
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "PositionEstimator.h"
#include "HeadingEstimator.h"
#include <kalman/ekfilter.hpp>


class OdometryKalmanNode
{
public:
	bool first;


	OdometryKalmanNode()
	{
		is_odom_initialised = false;
		current_heading = 0;
		first = false;
		filter = new PositionEstimator();

	}

	~OdometryKalmanNode()
	{

	}

	void processIMU(const sensor_msgs::Imu::ConstPtr& imu_msg)
	{
		double heading = tf::getYaw(imu_msg->orientation);

		heading = -(heading- M_PI/2);

		heading += north_correct;

		correct_angle(heading);

		if(is_imu_initialised)
		{
			ROS_DEBUG_NAMED("angle_estimator","before angle prediction: %.4f %.4f",current_heading,current_imu_cov);
			heading_estimator.predict(heading-previous_heading,0.000001);

			heading_estimator.get_result(current_heading,current_imu_cov);
			ROS_DEBUG_NAMED("angle_estimator","After angle prediction: %.4f %.4f",current_heading,current_imu_cov);
		}
		else
		{
			current_heading = heading;
			heading_estimator.init(current_heading,0.01);
			is_imu_initialised = true;
		}

		previous_heading = heading;

		if(is_odom_initialised)
		{
			publishEstimate();
		}

	}

	void processOdomControl(const nav_msgs::Odometry::ConstPtr& odom_msg)
	{
		Kalman::KVector<double,1,1> u(2);
		Kalman::KVector<double,1,1> state(3);

		if(is_odom_initialised)
		{
			u(1) = odom_msg->twist.twist.linear.x;
			u(2) = current_heading;

			ROS_DEBUG_NAMED("position_estimator","Control input: %.4f %.4f",u(1),u(2));
			state = filter->getX();
			ROS_DEBUG_NAMED("position_estimator","State before time update: %.4f %.4f %.4f",state(1),state(2),state(3));

			filter->timeUpdateStep(u);

			state = filter->getX();
			ROS_DEBUG_NAMED("position_estimator","State after time update: %.4f %.4f %.4f",state(1),state(2),state(3));



		}
		else
		{
			ROS_INFO_THROTTLE(1,"Waiting for GPS fix...");
		}

	}


	void processOdomUpdate(const nav_msgs::Odometry::ConstPtr& odom_msg)
	{
		Kalman::KVector<double,1,1> state(3);

		Kalman::KVector<double,1,1> z(2);


		if(is_odom_initialised)
		{


			double ds,dtheta_gps;
			double yaw_gps,gps_yaw_diff,vehicle_heading,vehicle_heading_cov;
			double cov_gps_heading;


			// get current heading from the angle estimator.
			heading_estimator.get_result(vehicle_heading,vehicle_heading_cov);
			// get current heading from gps
			yaw_gps = tf::getYaw(odom_msg->pose.pose.orientation);

			if(first==false && odom_msg->pose.covariance[35] < 9000)
			{
				first = true;
				heading_estimator.correct(yaw_gps,0.0001);
			}
			else
			{
				cov_gps_heading = odom_msg->pose.covariance[35];
				//handle forward reverse problematic
				gps_yaw_diff = yaw_gps - vehicle_heading;

				correct_angle(gps_yaw_diff);

				ROS_DEBUG_NAMED("angle_estimator","Gps_forward_backwards correction: %.4f, %.4f %.4f",yaw_gps,vehicle_heading,gps_yaw_diff);
				if(gps_yaw_diff > M_PI/2 )
				{
					yaw_gps -= M_PI;
				}
				if(gps_yaw_diff < -M_PI/2)
				{
					yaw_gps += M_PI;
				}
				ROS_DEBUG_NAMED("angle_estimator","Gps_forward_backwards correction stp2: %.4f, %.4f %.4f",yaw_gps,vehicle_heading,gps_yaw_diff);

				correct_angle(yaw_gps);

				heading_estimator.get_result(current_heading,current_imu_cov);
				ROS_DEBUG_NAMED("angle_estimator","Before Angle correction: %.4f %.4f %.4f %.4f",current_heading,current_imu_cov,yaw_gps,cov_gps_heading);
				if(cov_gps_heading < 999)
				{
					heading_estimator.correct(yaw_gps,cov_gps_heading);
				}

			}



			heading_estimator.get_result(current_heading,current_imu_cov);
			ROS_DEBUG_NAMED("angle_estimator","After Angle correction: %.4f %.4f",current_heading,current_imu_cov);
			try
			{
				listen.lookupTransform(base_frame,odom_msg->child_frame_id,ros::Time(0),transform);
			}
			catch(tf::TransformException& ex)
			{
				ROS_WARN_THROTTLE(1,"Could not transform gps measurements");
			}

			tf::Point p;

			p = transform.getOrigin();

			double diff_x = cos(current_heading)*p.x() - sin(current_heading)*p.y();
			double diff_y = sin(current_heading)*p.x() + cos(current_heading)*p.y();

			z(1) = odom_msg->pose.pose.position.x - diff_x;
			z(2) = odom_msg->pose.pose.position.y - diff_y;

			state = filter->getX();
			ROS_DEBUG_NAMED("position_estimator","State before measurment update: %.4f %.4f %.4f",state(1),state(2),state(3));

			filter->measureUpdateStep(z);

			state = filter->getX();
			ROS_DEBUG_NAMED("position_estimator","State after measurment update: %.4f %.4f %.4f",state(1),state(2),state(3));
		}
		else
		{
			// set inital pose and covarinace based on the first gps message
			Kalman::KVector<double,1,1> prior(3);
			Kalman::KMatrix<double,1,1> prior_cov(3,3);

			for(unsigned int i=1;i <= prior_cov.nrow();i++)
			{
				for(unsigned int j = 1; j<=prior_cov.ncol();j++)
				{
					if(i==j)
						prior_cov(i,j) = 1;
					else
						prior_cov(i,j) = 0;
				}
			}
			// do not update angle estimate with gps angle because we only have one message
			//if(sqrt( pow((odom_msg->pose.pose.position.x - prev_gps_msg.pose.pose.position.x),2) + pow(odom_msg->pose.pose.position.y - prev_gps_msg.pose.pose.position.y,2)) > 1.5 )
			//{
			//	ROS_DEBUG_NAMED("angle_estimator","Before angle correction: %.4f %.4f",current_heading,current_imu_cov);
			//	heading_estimator.correct(tf::getYaw(odom_msg->pose.pose.orientation),10);
			//}

			heading_estimator.get_result(current_heading,current_imu_cov);
			ROS_DEBUG_NAMED("angle_estimator","After angle correction: %.4f %.4f",current_heading,current_imu_cov);

			try
			{
				listen.lookupTransform(base_frame,odom_msg->child_frame_id,ros::Time(0),transform);
			}
			catch(tf::TransformException& ex)
			{
				ROS_WARN_THROTTLE(1,"Could not transform gps measurement");
			}

			tf::Point p;

			p = transform.getOrigin();

			double diff_x = cos(current_heading)*p.x() - sin(current_heading)*p.y();
			double diff_y = sin(current_heading)*p.x() + cos(current_heading)*p.y();

			prior(1) =  odom_msg->pose.pose.position.x - diff_x;
			prior(2) =  odom_msg->pose.pose.position.y - diff_y;
			prior(3) = current_heading;

			ROS_DEBUG_NAMED("position_estimator","Initialising position filter with x: %.4f y: %.4f theta: %.4f",prior(1),prior(2),prior(3));
			filter->setCovariance(gps_cov,imu_cov,odom_cov);
			filter->init(prior,prior_cov);

			is_odom_initialised = true;
		}

		prev_gps_msg = *odom_msg;

	}

	void publishEstimate()
	{
		Kalman::KVector<double,1,1> state(3);
		state = filter->getX();

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state(3));

		this->odom_trans.header.stamp = ros::Time::now();
		this->odom_trans.header.frame_id = odom_frame;
		this->odom_trans.child_frame_id = base_frame;
		this->odom_trans.transform.translation.x = state(1);
		this->odom_trans.transform.translation.y = state(2);
		this->odom_trans.transform.translation.z = 0.0;
		this->odom_trans.transform.rotation = odom_quat;

		this->odom_broadcaster.sendTransform(this->odom_trans);

		pub_odom_msg.header.stamp = ros::Time::now();

		pub_odom_msg.header.frame_id = odom_frame;
		pub_odom_msg.pose.pose.position.x = state(1);
		pub_odom_msg.pose.pose.position.y = state(2);

		pub_odom_msg.pose.pose.orientation = odom_quat;

		odom_est_pub.publish(pub_odom_msg);
	}

	/*!
	 * The publisher to publish the combined odometry estimate
	 */
	ros::Publisher odom_est_pub;

	/*!
	 * The transform broadcaster used for publishing the tf from odom_combined to base_link
	 * */
	tf::TransformBroadcaster odom_broadcaster;
	geometry_msgs::TransformStamped odom_trans;

	/*!
	 * The covariances used in the kalman filter for gps,imu,and odometry
	 * */
	double gps_cov,imu_cov,odom_cov;

	// \name Vectors
	//@{
	/*!
	 * GPS heading covariance calulation factors according to
	 * \f$ \sigma_{\theta} = \frac{ks}{\delta s} + |\delta \theta_gps| \cdot ktheta \f$.
	 * */
	double ktheta,ks;
	//@}
	double north_correct;

	double distance_threshold;

	std::string base_frame,gps_frame,odom_frame;

private:

	void correct_angle(double& angle)
	{
		while(angle > M_PI)
		{
			angle -= 2*M_PI;
		}

		while(angle < -M_PI)
		{
			angle += 2*M_PI;
		}
	}

	void calculate_delta_distance(double& ds,double& dtheta,const double& x,const double& y,const double& x_prev,const double& y_prev,const double& yaw,const double& prev_yaw)
	{
		ds =sqrt( pow((x - x_prev),2) + pow(y - y_prev,2));
		dtheta = yaw - prev_yaw;

		correct_angle(dtheta);

	}

	bool is_imu_initialised;
	bool is_odom_initialised;

	nav_msgs::Odometry prev_odom_msg;
	nav_msgs::Odometry prev_gps_msg;
	sensor_msgs::Imu prev_imu_msg;

	tf::TransformListener listen;
	tf::StampedTransform transform;



	PositionEstimator* filter;

	double current_heading;
	double current_imu_cov;

	double previous_heading;

	nav_msgs::Odometry pub_odom_msg;
	HeadingEstimator heading_estimator;



};





int main(int argc, char **argv)
{

	ros::init(argc, argv, "odometry_kalman_filter");

	ros::NodeHandle nh("~");
	ros::NodeHandle n;

	std::string imu_sub_top;
	std::string odom_sub_top;
	std::string odom_est_pub_top;
	std::string  gps_odom_sub_top;

	ros::Subscriber imu_sub;
	ros::Subscriber odom_sub;
	ros::Subscriber gps_sub;

	OdometryKalmanNode node;

	nh.param<std::string>("imu_subscriber_topic", imu_sub_top, "/fmSensors/IMU");
	nh.param<std::string>("odom_subscriber_topic", odom_sub_top, "/fmExtractors/wheel_odom");
	nh.param<std::string>("gps_odom_subscriber_topic", gps_odom_sub_top,"/fmExtractors/gps_odom");
	nh.param<std::string>("odom_estimate_publisher_topic", odom_est_pub_top,"/fmProcessors/odom_estimate");

	nh.param<std::string>("vehicle_frame",node.base_frame,"base_footprint");
	nh.param<std::string>("gps_frame",node.gps_frame,"gps_link");
	nh.param<std::string>("odom_estimate_frame",node.odom_frame,"odom_combined");

	nh.param<double>("gps_covariance",node.gps_cov,10);
	nh.param<double>("imu_covariance",node.imu_cov,0.01);
	nh.param<double>("odom_covariance",node.odom_cov,0.05);
	nh.param<double>("ks",node.ks,0.1); // how much to factor in positional movement into the gps angle covariance
	nh.param<double>("ktheta",node.ktheta,10); // how much to factor in gps heading change into the gps angle covariance
	nh.param<double>("magnetic_north_correction",node.north_correct,0.30); // offset between IMU magnetic north and gps north.
	nh.param<double>("angle_update_distance_threshold",node.distance_threshold,0.2);

	imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_sub_top.c_str(),10,&OdometryKalmanNode::processIMU,&node);
	odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_sub_top.c_str(),10,&OdometryKalmanNode::processOdomControl,&node);
	gps_sub = nh.subscribe<nav_msgs::Odometry>(gps_odom_sub_top.c_str(),10,&OdometryKalmanNode::processOdomUpdate,&node);

	node.odom_est_pub = n.advertise<nav_msgs::Odometry>(odom_est_pub_top.c_str(),5);

	ros::spin();
}
