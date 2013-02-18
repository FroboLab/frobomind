#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <msgs/row.h>
#include "visualization_msgs/Marker.h"


class KalmanRowEstimator
{
public:

	KalmanRowEstimator(){
		estimated_angle = 0.0;
		estimated_variance = 0.0;

		system_angle = 0.0;
		system_variance = 0.0;

		measurement_angle = 0.0;
		measurement_variance = 0.0;

		kalman_gain = 0.0;

		imu_initialised = false;

		imu_gyro_z_variance = 0.001;

	}

	virtual ~KalmanRowEstimator() {

	}

	void processIMUSystemUpdate(const sensor_msgs::ImuConstPtr& imu_msg) {
		if (!imu_initialised) {
			imu_initialised = true;
		} else {

			double ang = imu_msg->angular_velocity.z;

			ROS_INFO_NAMED("kalman_estimate","performing system update with: %.4f and %.4f",ang,imu_gyro_z_variance);
			system_update(ang, imu_gyro_z_variance);
			ROS_INFO_NAMED("kalman_estimate","Estimate after system update: %.4f 	%.4f",estimated_angle,estimated_variance);
		}
		prev_imu_msg = *imu_msg;

	}

	void processRowMeasurementUpdate(const msgs::rowConstPtr& row_msg) {

		double variance;
		double angle_rad;

		if(row_msg->leftvalid == 1 && row_msg->rightvalid == 1){

			if(row_msg->leftvar < row_msg->rightvar){
				angle_rad = row_msg->leftangle;
			}else{
				angle_rad = row_msg->rightangle;
			}

			variance = 2;

			ROS_INFO_NAMED("kalman_estimate","performing measurement update with: %.4f and %.4f",angle_rad,variance);
			measurement_update(angle_rad, variance);
			ROS_INFO_NAMED("kalman_estimate","Estimate after measurement: %.4f %.4f",estimated_angle,estimated_variance);
		}else if(row_msg->leftvalid == 1 && row_msg->rightvalid == 0){


			angle_rad = row_msg->leftangle;


			variance = 4;

			ROS_INFO_NAMED("kalman_estimate","performing measurement update with: %.4f and %.4f",angle_rad,variance);
			measurement_update(angle_rad, variance);
			ROS_INFO_NAMED("kalman_estimate","Estimate after measurement: %.4f %.4f",estimated_angle,estimated_variance);
		}else if(row_msg->leftvalid == 0 && row_msg->rightvalid == 1){

			angle_rad = row_msg->rightangle;
			variance = 4;
			ROS_INFO_NAMED("kalman_estimate","performing measurement update with: %.4f and %.4f",angle_rad,variance);
			measurement_update(angle_rad, variance);
			ROS_INFO_NAMED("kalman_estimate","Estimate after measurement: %.4f %.4f",estimated_angle,estimated_variance);
		}else{
			angle_rad = 0;
			variance = 400;
			ROS_INFO_NAMED("kalman_estimate","performing measurement update with: %.4f and %.4f",angle_rad,variance);
			measurement_update(angle_rad, variance);
			ROS_INFO_NAMED("kalman_estimate","Estimate after measurement: %.4f %.4f",estimated_angle,estimated_variance);
		}

		msg = *row_msg;

		msg.leftangle = estimated_angle;
		msg.rightangle = estimated_angle;

		row_pub.publish(msg);

		//doublevariance = (256 - row_msg->quality) * quality_to_variance;
		//double angle_rad = row_msg->heading * M_PI / 180.0;

		//row_offset = row_msg->offset;



	}

	double imu_gyro_z_variance;
	ros::Publisher marker_pub;
	ros::Publisher row_pub;

private:

	void system_update(double angle,double variance)
	{

		// make new angle prediction
		system_angle = estimated_angle + (angle*(1/75));
		system_variance = estimated_variance + variance;

		// correct for angle "overflow"
		correct_angle(system_angle);

		// update estimate with prediction
		estimated_angle = system_angle;
		estimated_variance = system_variance;

		marker.id = 1;

		marker.scale.x = (0.1);

		marker.color.b = 1.0;
		marker.color.r = 0;
		marker.color.a = 0.6;

		geometry_msgs::Point p;
		marker.points.clear();

		p.x = cos(estimated_angle)*-10; p.y = sin(estimated_angle)*-10; p.z = 0;

		marker.points.push_back(p);

		p.x = cos(estimated_angle)*10; p.y = sin(estimated_angle)*10; p.z = 0;

		marker.points.push_back(p);
		marker_pub.publish(marker);

	}

	void measurement_update(double angle, double variance)
	{

		correct_angle(measurement_angle);

		// update estimate with measurement
		kalman_gain = system_variance / (system_variance + variance);
		measurement_angle = system_angle + kalman_gain * (angle - system_angle);
		measurement_variance = system_variance * (1 - kalman_gain);

		correct_angle(measurement_angle);

		estimated_angle = measurement_angle;
		estimated_variance = measurement_variance;


		marker.header.frame_id = "/laser_link";
		marker.ns = "Ransac";
		marker.pose.orientation.w = 1.0;
		marker.type = visualization_msgs::Marker::LINE_STRIP;

		marker.id = 1;

		marker.scale.x = (0.1);

		marker.color.b = 1.0;
		marker.color.r = 0;
		marker.color.a = 0.6;

		geometry_msgs::Point p;
		marker.points.clear();

		p.x = cos(estimated_angle)*-10; p.y = sin(estimated_angle)*-10; p.z = 0;

		marker.points.push_back(p);

		p.x = cos(estimated_angle)*10; p.y = sin(estimated_angle)*10; p.z = 0;

		marker.points.push_back(p);
		marker_pub.publish(marker);

		//msg.header.stamp = ros::Time::now();
		//msg.header.seq++;

		//msg.heading = estimated_angle/M_PI * 180;
		//msg.offset = row_offset;
		//msg.quality = estimated_variance;

		//row_est_pub.publish(msg);
	}

	void correct_angle(double& angle)
	{
		if(angle >= M_PI)
		{
			angle -= 2*M_PI;
		}
		else if (angle <= -M_PI)
		{
			angle += 2*M_PI;
		}
	}



	double estimated_angle;
	double estimated_variance;

	double system_angle;
	double system_variance;

	double measurement_angle;
	double measurement_variance;

	double kalman_gain;

	sensor_msgs::Imu prev_imu_msg;
	msgs::row msg;
	visualization_msgs::Marker marker;
	bool imu_initialised;


};



int main(int argc, char **argv) {

	ros::init(argc, argv, "kalman_row_estimator");

	ros::NodeHandle nh("~");
	ros::NodeHandle n;

	std::string imu_s_t;
	std::string row_s_t;
	std::string row_est_p_t;

	ros::Subscriber imu_s;
	ros::Subscriber row_s;

	KalmanRowEstimator krf;

	nh.param<std::string>("imu_subscriber_topic", imu_s_t, "/fmSensors/IMU");
	nh.param<std::string>("row_subscriber_topic", row_s_t, "/fmExtractors/rows");
	nh.param<std::string>("row_estimate_publisher_topic", row_est_p_t,"/fmProcessors/row_estimate");

	imu_s = nh.subscribe<sensor_msgs::Imu>(imu_s_t.c_str(),1, &KalmanRowEstimator::processIMUSystemUpdate,&krf);
	row_s = nh.subscribe<msgs::row>(row_s_t.c_str(),1, &KalmanRowEstimator::processRowMeasurementUpdate,&krf);


	krf.row_pub = nh.advertise<msgs::row>(row_est_p_t,1);
	krf.marker_pub = nh.advertise<visualization_msgs::Marker>("kalman_row_marker", 1);


	ros::spin();

}
