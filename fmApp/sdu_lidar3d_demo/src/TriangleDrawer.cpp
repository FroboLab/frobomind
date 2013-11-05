/*
 * TriangleDrawer.cpp
 *
 *  Created on: Aug 1, 2013
 *      Author: kent
 */

#include "TriangleDrawer.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "TriangleDrawer");

	TriangleDrawer rowNode;
	rowNode.makeItSpin();

	return 0;
}

TriangleDrawer::TriangleDrawer()
{
	//	ROS node handler stuff
	this->nodeHandler = ros::NodeHandle("~");
	this->nodeHandler.param<int>("loopRate", this->loopRate, 100);

	//	Setup laser scanner --> point cloud stuff
	this->tfListener.setExtrapolationLimit(ros::Duration(0.1));


	this->markerPublisher = this->nodeHandler.advertise<visualization_msgs::Marker>("line_marker_pub", 10);
	this->closePointsPublisher = this->nodeHandler.advertise<sensor_msgs::PointCloud2>("close_points", 10);

	// Setup system output
	this->nodeHandler.param<std::string>("rowTopic", this->output.rowTopic, "rowTopic");
	this->output.rowPublisher = this->nodeHandler.advertise<msgs::row>(this->output.rowTopic, 10);

	//	Setup system input
	//this->nodeHandler.param<std::string>("scanTopic", this->input.scanTopic, "/fmSensors/laser_msg");
	this->nodeHandler.param<std::string>("scanTopic", this->input.scanTopic, "/scan");
	this->nodeHandler.param<std::string>("scanLink", this->input.scanLink, "/laser");
	this->input.scanSubscribe = this->nodeHandler.subscribe<sensor_msgs::LaserScan>(this->input.scanTopic, 10, &TriangleDrawer::laserScanCallback, this);


	// List of markers
	visualization_msgs::Marker triangle_list;
	triangle_list.header.frame_id = "/base_link";
	triangle_list.action = visualization_msgs::Marker::ADD;
	triangle_list.pose.orientation.w = 1.0;
	triangle_list.type = visualization_msgs::Marker::TRIANGLE_LIST;
	triangle_list.scale.x = 1.0;
	triangle_list.scale.y = 1.0;
	triangle_list.scale.z = 1.0;
	triangle_list.color.a = 0.4;
	triangle_list.color.r = 0.4;
	triangle_list.color.g = 0.3;
	triangle_list.color.b = 0.9;

	// Read parameters from parameter server if available otherwise use default values
	this->nodeHandler.param<double>("minDist", this->minDist, 0.01);
	this->nodeHandler.param<double>("maxDist", this->maxDist, 1.00);
}

TriangleDrawer::~TriangleDrawer()
{
	// TODO Auto-generated destructor stub
}

void TriangleDrawer::makeItSpin()
{
	ros::Rate r(this->loopRate);
	int counter = 0;

	while (ros::ok() && this->nodeHandler.ok())
	{
		ros::spinOnce();
		counter++;
		publishTriangleSurfaces(counter);
		publishPointsCloseToScanner();
		r.sleep();
	}
}

void TriangleDrawer::publishTriangleSurfaces(int counter)
{
	// Triangle list
	// =============
	triangle_list.header.frame_id = "/base_link";
	triangle_list.action = visualization_msgs::Marker::ADD;
	triangle_list.pose.orientation.w = 1.0;
	triangle_list.type = visualization_msgs::Marker::TRIANGLE_LIST;
	triangle_list.scale.x = 1.0;
	triangle_list.scale.y = 1.0;
	triangle_list.scale.z = 1.0;
	triangle_list.color.a = 0.4;
	triangle_list.color.r = 0.4;
	triangle_list.color.g = 0.3;
	triangle_list.color.b = 0.9;
	triangle_list.header.stamp = ros::Time::now();
	triangle_list.id = counter % 100;
	triangle_list.points.clear();


	// ROS_INFO("Number of points: %d and %d", this->input.pointCloud.size(), this->input.pointCloudPrevious.size());
	if (this->input.pointCloud.size() > 270 && this->input.pointCloudPrevious.size() > 270)
	{
		//ROS_INFO("Points ...");
		for(int k = 0; k < 270; k = k + 10)
		{
			this->addPointFromCurrentScan(k + 1);
			this->addPointFromPreviousScan(k + 1);
			this->addPointFromPreviousScan(k + 2);
			this->addPointFromCurrentScan(k + 1);
			this->addPointFromCurrentScan(k + 2);
			this->addPointFromPreviousScan(k + 2);
		}
	}
	else
	{
		//ROS_INFO("No points ...");
	}

	char ns2[255];
	sprintf(ns2, " %d", 5);
	std::string name = "Testing";
	name.append(ns2);
	triangle_list.ns = name.c_str();
	this->markerPublisher.publish(triangle_list);

}

void TriangleDrawer::publishPointsCloseToScanner()
{
	// Close points
	// ============

	sensor_msgs::PointCloud2 pointCloud;

	pcl::PointCloud<PointT> pCloud = this->input.pointCloud;
	pCloud.clear();
	PointT p;
	double l;

	if (this->input.pointCloud.size())
	{
		//	Pre-Filter data
		for (int i = 0; i < this->input.pointCloud.size(); i++)
		{
			p = this->input.pointCloud.at(i);
			l = std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2) + std::sqrt(std::pow(p.z, 2)));

			if (minDist < l && l < maxDist) 
			{	
				pCloud.push_back(p);
			}
		}
	}

	pcl::toROSMsg(pCloud, pointCloud);
	this->closePointsPublisher.publish(pointCloud);
}

void TriangleDrawer::addPointFromCurrentScan(int index)
{
	if (this->input.pointCloud.size())
	{
		PointT p;
		geometry_msgs::Point p0;
		p = this->input.pointCloud.at(index);
		//std::cout << "x: " << p.x << "\ty: " << p.y << "\tz: " << p.z << std::endl;
		p0.x = p.x;
		p0.y = p.y;
		p0.z = p.z;
		triangle_list.points.push_back(p0);
	}
}

void TriangleDrawer::addPointFromPreviousScan(int index)
{
	if (this->input.pointCloudPrevious.size())
	{
		PointT p;
		geometry_msgs::Point p0;
		p = this->input.pointCloudPrevious.at(index);
		//std::cout << "x: " << p.x << "\ty: " << p.y << "\tz: " << p.z << std::endl;
		p0.x = p.x;
		p0.y = p.y;
		p0.z = p.z;
		triangle_list.points.push_back(p0);
	}
}

void TriangleDrawer::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
	try
	{
//		ros::Time now = ros::Time::now();
//		this->tfListener.waitForTransform("/laser", "/base_link",
//				      now, ros::Duration(3.0));
//
//		std::cout << "Got transform" << std::endl;

		input.pointCloudPrevious = input.pointCloud;

		//	Transform laser scan to point cloud in the scanners own frame
		sensor_msgs::PointCloud2 pointCloud;
		//this->laserProjection.transformLaserScanToPointCloud(this->input.scanLink, *data, pointCloud, this->tfListener);
		this->laserProjection.transformLaserScanToPointCloud("/base_link", *data, pointCloud, this->tfListener);

		pcl::fromROSMsg(pointCloud, this->input.pointCloud);
	}
	catch(...)
	{
	}
}

