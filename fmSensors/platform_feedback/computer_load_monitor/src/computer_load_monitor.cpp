/*
 * monitor.cpp
 *
 *  Created on: Apr 20, 2013
 *      Author: kent
 *  Modified Feb 20, 2014, Kjeld Jensen, profiled, documented, adapted to FroboMind naming conventions. 
 */

#include <fstream>
#include <string>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <msgs/FloatStamped.h>

#define MEMORY_load "/proc/meminfo"
#define CPU_load "/proc/stat"

std::string readFile(const char* file);
double getMemoryload(std::string buffer);
double getCpuload(std::string buffer);

int main (int argc, char** argv)
{
	//	Ros init
	ros::init(argc, argv, "computer_load_monitor");

	//	Node handler
	int update_rate;
	ros::NodeHandle nodeHandler("~");
	nodeHandler.param<int>("update_rate", update_rate, 5);
	ros::Rate loopRate(update_rate);

	//	loads (CPU/Memory)
	ros::Publisher memloadPub = nodeHandler.advertise<msgs::FloatStamped>("/fmInformation/memory_load", 5);
	msgs::FloatStamped memMsg;

	ros::Publisher cpuloadPub = nodeHandler.advertise<msgs::FloatStamped>("/fmInformation/cpu_load", 5);
	msgs::FloatStamped cpuMsg;

	//	Monitor message
	// ROS_INFO(" Monitoring CPU and memory load ...");

	while (ros::ok() && nodeHandler.ok())
	{
		//	Publish memory load
		memMsg.header.stamp = ros::Time::now();
		memMsg.data = getMemoryload(readFile(MEMORY_load));
		memloadPub.publish(memMsg);	//	Output is in range 0.0 to 1.0 as memory load

		//	Publish cpu load
		cpuMsg.header.stamp = ros::Time::now();
		cpuMsg.data = getCpuload(readFile(CPU_load));
		cpuloadPub.publish(cpuMsg);	//	Output is in range 0.0 to 1.0 as cpu load since last publish (averaged) across all siblings

		loopRate.sleep();
	}

	return 0;
}

std::string readFile(const char* file)
{
	std::ifstream fileContent(file, std::ios::in | std::ios::binary);
	std::string temp, buffer = "";

	if (fileContent)
	{
		//	Read file
		while (std::getline(fileContent, temp)) buffer.append(temp);

		//	Close file
		fileContent.close();
	}

	return buffer;
}

double getMemoryload(std::string buffer)
{
	size_t totalPos = buffer.find("MemTotal:");
	size_t freePos = buffer.find("MemFree");
	std::string temp;

	temp = buffer.substr(totalPos + 9, 15);
	double totalMem = atof(temp.c_str());

	temp = buffer.substr(freePos + 9, 15);
	double freeMem = atof(temp.c_str());

	return ((totalMem - freeMem) / totalMem);
	//return (totalMem - freeMem);
}

double getCpuload(std::string buffer)
{
	//	Hold jiffies
	static std::vector<double> oj(10, 0);
	std::vector<double> nj;

	//	Get relevant jiffies
	size_t pos = buffer.find("cpu0");
	std::string temp = buffer.substr(3, pos - 1); // extract the line beginning with 'cpu'
	std::stringstream ss(temp);

	//	Put string stream into vector
	std::copy(std::istream_iterator<double>(ss), std::istream_iterator<double>(), std::back_inserter(nj));

	//	Calculate (total) cpu load as the amount of busy time divided by the total time since last update.
	// for more info about the values 0,...,6 please see http://man7.org/linux/man-pages/man5/proc.5.html
	double loadAverage = 	( (nj[0] + nj[1] + nj[2] + nj[4] + nj[5] + nj[6]) - (oj[0] + oj[1] + oj[2] + oj[4] + oj[5] + oj[6]) ) /
							( (nj[0] + nj[1] + nj[2] + nj[3] + nj[4] + nj[5] + nj[6]) - (oj[0] + oj[1] + oj[2] + oj[3] + oj[4] + oj[5] + oj[6]) );

	oj = nj;	//	Push data back

	return loadAverage;
}
