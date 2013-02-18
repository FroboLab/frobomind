/*
 * SparkFun9DOF.cpp
 *
 *  Created on: Mar 13, 2012
 *      Author: morl
 */

#include "SparkFun9DOF.h"

#define DEG2RAD M_PI/180.0

SparkFun9DOF::SparkFun9DOF()
{
	// TODO Auto-generated constructor stub

}

SparkFun9DOF::~SparkFun9DOF()
{
	// TODO Auto-generated destructor stub
}

void SparkFun9DOF::enableAccelerometer(bool yes)
{
	is_enabled_acc_ = yes;
}

void SparkFun9DOF::enableGyro(bool yes)
{
	is_enabled_gyro_ = yes;
}

void SparkFun9DOF::enableMag(bool yes)
{
	is_enabled_mag_ = yes;
}

void SparkFun9DOF::setFrameId(string frame_id)
{
	frame_id_ = frame_id;
}

void SparkFun9DOF::newMsgCallback(const msgs::serial::ConstPtr& msg)
{

	boost::char_separator<char> sep("$*,");
	tokenizer::iterator tok_iter;
	tokenizer tokens(msg->data, sep);

	tok_iter = tokens.begin();
	// get NMEA identifier
	if((*tok_iter).compare("SFIMU") == 0)
	{
		parseIMU(tokens,msg->data);

	}
	else if((*tok_iter).compare("SFMAG") == 0)
	{
		parseMAG(tokens,msg->data);
	}
	else
	{
		ROS_INFO("Ignoring Unknown NMEA identifier");
	}

}

void SparkFun9DOF::publishMag()
{
	++msg_mag_.header.seq;
	msg_mag_.header.frame_id = frame_id_;
	msg_mag_.header.stamp = ros::Time::now();
	msg_mag_.x = m_x;
	msg_mag_.y = m_y;
	msg_mag_.z = m_z;

	pub_mag_.publish(msg_mag_);
}

void SparkFun9DOF::publishAcc()
{
	++msg_acc_.header.seq;
	msg_acc_.header.frame_id = frame_id_;
	msg_acc_.header.stamp = ros::Time::now();
	msg_acc_.x = a_x;
	msg_acc_.y = a_y;
	msg_acc_.z = a_z;

	pub_acc_.publish(msg_acc_);
}

void SparkFun9DOF::publishGyro()
{
	++msg_gyro_.header.seq;
	msg_gyro_.header.frame_id = frame_id_;
	msg_gyro_.header.stamp = ros::Time::now();
	msg_gyro_.x = g_x;
	msg_gyro_.y = g_y;
	msg_gyro_.z = g_z;

	pub_gyro_.publish(msg_gyro_);
}

void SparkFun9DOF::parseIMU(tokenizer& tokens,string raw)
{
	unsigned int chk,calc_chk;
	tokenizer::iterator tok_iter = tokens.begin();

	// skip NMEA header
	tok_iter++;

	if( countTokens(tokens) == 8 )
	{
		// extract and verify checksum
		chk = extractChecksum(tokens,7);
		calc_chk = calculateChecksum(raw);
		if(chk != calc_chk)
		{
			ROS_WARN("IMU checksum did not match, skipping measurement");
		}
		else
		{
			try
			{
				a_x = boost::lexical_cast<int>(*tok_iter++) / 1000.0 * 4 * 9.82;
				a_y = boost::lexical_cast<int>(*tok_iter++) / 1000.0 * 4 * 9.82;
				a_z = boost::lexical_cast<int>(*tok_iter++) / 1000.0 * 4 * 9.82;

				g_x = boost::lexical_cast<int>(*tok_iter++) * 1/14.375 * DEG2RAD;
				g_y = boost::lexical_cast<int>(*tok_iter++) * 1/14.375 * DEG2RAD;
				g_z = boost::lexical_cast<int>(*tok_iter++) * 1/14.375 * DEG2RAD;

				if(is_enabled_acc_)
				{
					publishAcc();
				}
				if(is_enabled_gyro_)
				{
					publishGyro();
				}
			}
			catch(boost::bad_lexical_cast &)
			{
				ROS_ERROR("Could not convert accelerometer and gyro readings");
			}
		}
	}
	else
	{
		ROS_ERROR("Unexpected number of tokens in SPIMU message");
	}

}

void SparkFun9DOF::parseMAG(tokenizer& tokens,string raw)
{
	unsigned int chk,calc_chk;
	tokenizer::iterator tok_iter = tokens.begin();

	// skip NMEA header
	tok_iter++;

	if( countTokens(tokens) == 5 )
	{
		// extract and verify checksum
		chk = extractChecksum(tokens,4);
		calc_chk = calculateChecksum(raw);
		if(chk != calc_chk)
		{
			ROS_WARN("MAG checksum did not match, skipping measurement");
		}
		else
		{
			try
			{
				m_x = boost::lexical_cast<int>(*tok_iter++) / 660.0;
				m_y = boost::lexical_cast<int>(*tok_iter++) / 660.0;
				m_z = boost::lexical_cast<int>(*tok_iter++) / 660.0;

				if(is_enabled_mag_)
				{
					publishMag();
				}
			}
			catch (boost::bad_lexical_cast &)
			{
				ROS_ERROR("Could not convert Magnetormeter readings");
			}
		}
	}
	else
	{
		ROS_ERROR("Unexpected number of tokens in SPIMU message");
	}
}

unsigned int SparkFun9DOF::extractChecksum(tokenizer tokens,int chk_msg_start)
{
	tokenizer::iterator tok_iter_chk;
	unsigned int chk;

	tok_iter_chk = tokens.begin();

	for(int j=0;j<chk_msg_start;j++)
	{
		tok_iter_chk++;
	}

	std::stringstream ss(*tok_iter_chk);
	ss >> std::hex >> chk;

	return chk;
}

unsigned int SparkFun9DOF::calculateChecksum(string s)
{
	uint8_t chk = 0;

	for(unsigned int i=1;i<s.length();i++)
	{
		if(s.at(i) == '*')
		{
			break;
		}
		chk ^= (uint8_t)s.at(i);
	}

	return chk;
}

void SparkFun9DOF::setGyroTopic(ros::Publisher pub)
{
	pub_gyro_ = pub;
}

void SparkFun9DOF::setAccTopic(ros::Publisher pub)
{
	pub_acc_ = pub;
}

void SparkFun9DOF::setMagTopic(ros::Publisher pub)
{
	pub_mag_ = pub;
}

unsigned int SparkFun9DOF::countTokens(tokenizer& tokens)
{
	tokenizer::iterator tok_iter;
	unsigned int count = 0;
	for (tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter)
	{
		count++;
	}

	return count;
}





