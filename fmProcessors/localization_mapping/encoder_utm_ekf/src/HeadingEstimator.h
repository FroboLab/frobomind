/*
 * HeadingEstimator.h
 *
 *  Created on: May 2, 2012
 *      Author: morl
 */

#ifndef HEADINGESTIMATOR_H_
#define HEADINGESTIMATOR_H_

class HeadingEstimator
{
public:
	HeadingEstimator();
	virtual ~HeadingEstimator();

	void init(double initial_heading,double initial_covariance);

	void predict(double d_theta_imu,double d_theta_cov);
	void correct(double theta_gps,double theta_gps_cov);

	void get_result(double& posterior_mean,double&posterior_cov);

private:

	void correct_angle_overflow(double& angle);

	double heading_mean_,heading_cov_;
	double K;


};

#endif /* HEADINGESTIMATOR_H_ */
