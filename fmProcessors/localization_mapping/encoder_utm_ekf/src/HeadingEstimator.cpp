/*
 * HeadingEstimator.cpp
 *
 *  Created on: May 2, 2012
 *      Author: morl
 */

#include "HeadingEstimator.h"
#include <math.h>

HeadingEstimator::HeadingEstimator()
{
	// TODO Auto-generated constructor stub
	heading_cov_ = 0;
	heading_mean_ = 0;
	K = 0;
}

HeadingEstimator::~HeadingEstimator()
{
	// TODO Auto-generated destructor stub
}

void HeadingEstimator::init(double initial_heading, double initial_covariance)
{
	heading_cov_ = initial_covariance;
	heading_mean_ = initial_heading;
}

void HeadingEstimator::predict(double d_theta_imu, double d_theta_cov)
{
	heading_mean_ += d_theta_imu;
	heading_cov_ = heading_cov_ + d_theta_cov;

	correct_angle_overflow(heading_mean_);
}

void HeadingEstimator::correct(double theta_gps, double theta_gps_cov)
{

	K = heading_cov_ / (heading_cov_ + theta_gps_cov);

	double diff = theta_gps - heading_mean_;

	correct_angle_overflow(diff);

	heading_mean_ = heading_mean_ + K * (diff);

	correct_angle_overflow(heading_mean_);

	heading_cov_ = (1 - K) * heading_cov_;

}

void HeadingEstimator::get_result(double& posterior_mean, double& posterior_cov)
{
	posterior_mean = heading_mean_;
	posterior_cov = heading_cov_;
}

void HeadingEstimator::correct_angle_overflow(double& angle)
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



