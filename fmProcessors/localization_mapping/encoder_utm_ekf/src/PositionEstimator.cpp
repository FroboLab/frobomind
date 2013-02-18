	/*
 * PositionEstimator.cpp
 *
 *  Created on: Apr 26, 2012
 *      Author: morl
 */

#include "PositionEstimator.h"
#include <math.h>
#include <ros/console.h>

PositionEstimator::PositionEstimator()
{

	// state size
	// input vector
	// process noise
	// measurement size
	// measurement noise
	setDim(3,2,3,2,2);



}

PositionEstimator::~PositionEstimator()
{
	// TODO Auto-generated destructor stub
}

void PositionEstimator::makeA()
{
	A(1,1) = 1;
	A(1,2) = 0;
	A(1,3) = -u(1)*sin(x(3));
	A(2,1) = 0;
	A(2,2) = 1;
	A(2,3) = u(1) * cos(x(3));
	A(3,1) = 0;
	A(3,2) = 0;
	A(3,3) = 1;

	//ROS_ERROR("Make A: %.4f %.4f",A(1,3),A(2,3));
}

void PositionEstimator::makeV()
{
	V(1,1) = 1;
	V(1,2) = 0;
	//V(1,3) = 0;
	V(2,1) = 0;
	V(2,2) = 1;
//	V(2,3) = 0;
//	V(3,1) = 0;
//	V(3,2) = 0;
//	V(3,3) = 1;
}

void PositionEstimator::makeQ()
{
	Q(1,1) = odom_var;
	Q(1,2) = 0;
	Q(1,3) = 0;
	Q(1,2) = 0;
	Q(2,2) = odom_var;
	Q(2,3) = 0;
	Q(3,1) = 0;
	Q(3,2) = 0;
	Q(3,3) = imu_var;
}

void PositionEstimator::makeR()
{
	R(1,1) = gps_var;
	R(1,2) = 0;
	//R(1,3) = 0;
	R(2,1) = 0;
	R(2,2) = gps_var;
//	R(2,3) = 0;
//	R(3,1) = 0;
//	R(3,2) = 0;
//	R(3,3) = 999999;
}

void PositionEstimator::makeH()
{
//	H(1,1) = 1;
//	H(1,2) = 0;
//	H(1,3) = 0;
//	H(2,1) = 0;
//	H(2,2) = 1;
//	H(2,3) = 0;
//	H(3,1) = 0;
//	H(3,2) = 0;
//	H(3,3) = 1;

	H(1,1) = 1;
	H(1,2) = 0;
	H(1,3) = 0;
	H(2,1) = 0;
	H(2,2) = 1;
	H(2,3) = 0;
}

void PositionEstimator::makeW()
{
	W(1,1) = 1;
	W(1,2) = 0;
	W(1,3) = 0;
	W(2,1) = 0;
	W(2,2) = 1;
	W(2,3) = 0;
	W(3,1) = 0;
	W(3,2) = 0;
	W(3,3) = 1;
}

void PositionEstimator::makeProcess()
{
	Vector x_new(x.size());

	//ROS_ERROR("U is: %.4f %.4f",u(1),u(2));

	x_new(1) = x(1) + cos(x(3))*u(1);
	x_new(2) = x(2) + sin(x(3))*u(1);
	x_new(3) = u(2);
	// wrap
	if(x_new(3) > M_PI)
	{
		x_new(3) -= 2*M_PI;
	}
	else if(x_new(3) < -M_PI)
	{
		x_new(3) += 2* M_PI;
	}

	//ROS_ERROR("Make Process: %.4f %.4f %.4f",x_new(1),x_new(2),x_new(3));

	x.swap(x_new);
}

void PositionEstimator::setCovariance(double gps_cov, double imu_cov,
		double odom_cov)
{

	gps_var = gps_cov;
	imu_var = imu_cov;
	odom_var = odom_cov;

}

void PositionEstimator::makeMeasure()
{
	Vector z_est(z.size());
	z_est(1) = x(1);
	z_est(2) = x(2);
	//z_est(3) = x(3);
	//ROS_ERROR("Make Measure: %.4f %.4f",z_est(1),z_est(2));
	z.swap(z_est);

}


