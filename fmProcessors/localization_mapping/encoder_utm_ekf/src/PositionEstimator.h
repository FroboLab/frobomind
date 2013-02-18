/*
 * PositionEstimator.h
 *
 *  Created on: Apr 26, 2012
 *      Author: morl
 */

#ifndef POSITIONESTIMATOR_H_
#define POSITIONESTIMATOR_H_

#include <kalman/ekfilter.hpp>

class PositionEstimator : public Kalman::EKFilter<double,1,false,false,true>
{
public:
	PositionEstimator();
	virtual ~PositionEstimator();

	void setCovariance(double gps_cov,double imu_cov, double odom_cov);
protected:
	void makeA();
	void makeV();
	void makeQ();
	void makeR();
	void makeH();
	void makeW();

	void makeProcess();
	void makeMeasure();
private:
	double gps_var,imu_var,odom_var;
};

#endif /* POSITIONESTIMATOR_H_ */
