/*
 * monitor.cpp
 *
 *  Created on: Apr 20, 2013
 *      Author: kent
 */

#include <fstream>
#include <string>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <msgs/FloatStamped.h>

#define MEMORY_USAGE "/proc/meminfo"
#define CPU_USAGE "/proc/stat"
#define CPU_INFO "/proc/cpuinfo"

std::string readFile(const char* file);
double getMemoryUsage(std::string buffer);
double getCpuUsage(std::string buffer);

int main (int argc, char** argv)
{
	//	Ros init
	ros::init(argc, argv, "usageMonitor");

	//	Node handler
	int rate;
	ros::NodeHandle nodeHandler("~");
	nodeHandler.param<int>("rate", rate, 4);
	ros::Rate loopRate(rate);

	//	Usages (CPU/Memory)
	ros::Publisher memUsagePub = nodeHandler.advertise<msgs::FloatStamped>("memory", 5);
	msgs::FloatStamped memMsg;

	ros::Publisher cpuUsagePub = nodeHandler.advertise<msgs::FloatStamped>("cpu", 5);
	msgs::FloatStamped cpuMsg;

	//	Monitor message
	ROS_INFO(" Monitoring CPU and memory usage ...");

	while (ros::ok() && nodeHandler.ok())
	{
		//	Publish memory usage
		memMsg.header.stamp = ros::Time::now();
		memMsg.data = getMemoryUsage(readFile(MEMORY_USAGE));
		memUsagePub.publish(memMsg);	//	Output is in range 0.0 to 1.0 as memory usage

		//	Publish cpu usage
		cpuMsg.header.stamp = ros::Time::now();
		cpuMsg.data = getCpuUsage(readFile(CPU_USAGE));
		cpuUsagePub.publish(cpuMsg);	//	Output is in range 0.0 to 1.0 as average cpu usage across all siblings

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

double getMemoryUsage(std::string buffer)
{
	size_t totalPos = buffer.find("MemTotal:");
	size_t freePos = buffer.find("MemFree");
	std::string temp;

	temp = buffer.substr(totalPos + 9, 15);
	double totalMem = atof(temp.c_str());

	temp = buffer.substr(freePos + 9, 15);
	double freeMem = atof(temp.c_str());

	return ((totalMem - freeMem) / totalMem);
}

double getCpuUsage(std::string buffer)
{
	//	Hold jiffies
	static std::vector<double> oj(10, 0);
	std::vector<double> nj;

	//	Get relevant jiffies
	size_t pos = buffer.find("cpu0");
	std::string temp = buffer.substr(3, pos - 1);
	std::stringstream ss(temp);

	//	Put string stream into vector
	std::copy(std::istream_iterator<double>(ss), std::istream_iterator<double>(), std::back_inserter(nj));

	//	Calculate load average
	double loadAverage = 	( (nj[0] + nj[1] + nj[2] + nj[4] + nj[5] + nj[6]) - (oj[0] + oj[1] + oj[2] + oj[4] + oj[5] + oj[6]) ) /
							( (nj[0] + nj[1] + nj[2] + nj[3] + nj[4] + nj[5] + nj[6]) - (oj[0] + oj[1] + oj[2] + oj[3] + oj[4] + oj[5] + oj[6]) );

	oj = nj;	//	Push data back

	return loadAverage;
}
