/*
 * SparkFun9DOF.h
 *
 *  Created on: Mar 13, 2012
 *      Author: morl
 */

#ifndef SPARKFUN9DOF_H_
#define SPARKFUN9DOF_H_
#include <ros/ros.h>
#include <ros/console.h>

#include <msgs/accelerometer.h>
#include <msgs/gyroscope.h>
#include <msgs/magnetometer.h>
#include <msgs/serial.h>

#include <string>

#include "boost/tokenizer.hpp"
#include "boost/lexical_cast.hpp"

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;


using namespace std;
class SparkFun9DOF
{
public:
	SparkFun9DOF();
	virtual ~SparkFun9DOF();
	void enableAccelerometer(bool yes);
	void enableGyro(bool yes);
	void enableMag(bool yes);

	void setFrameId(string frame_id);
	void setGyroTopic(ros::Publisher pub);
	void setAccTopic(ros::Publisher pub);
	void setMagTopic(ros::Publisher pub);

	void newMsgCallback(const msgs::serial::ConstPtr& msg);


private:
	bool is_enabled_mag_,is_enabled_gyro_,is_enabled_acc_;

	double a_x,a_y,a_z,g_x,g_y,g_z,m_x,m_y,m_z;

	void publishMag();
	void publishAcc();
	void publishGyro();

	void parseIMU(tokenizer& tokens,string raw);
	void parseMAG(tokenizer& tokens,string raw);

	unsigned int extractChecksum(tokenizer tokens,int chk_msg_start);
	unsigned int calculateChecksum(string s);
	unsigned int countTokens(tokenizer& tokens);

	msgs::gyroscope msg_gyro_;
	msgs::accelerometer msg_acc_;
	msgs::magnetometer msg_mag_;

	ros::Publisher pub_acc_,pub_mag_,pub_gyro_;

	string frame_id_;

};

#endif /* SPARKFUN9DOF_H_ */
