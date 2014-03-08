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
	triangle_list.color.a = 1.0;
	triangle_list.color.r = 165 / 255.;
	triangle_list.color.g = 32 / 255.;
	triangle_list.color.b = 25 / 255.;
	triangle_list.header.stamp = ros::Time::now();
	triangle_list.id = counter % 100;
	triangle_list.points.clear();
	triangle_list.colors.clear();

	// ROS_INFO("Number of points: %d and %d", this->input.pointCloud.size(), this->input.pointCloudPrevious.size());
	if (this->input.pointCloud.size() > 270 && this->input.pointCloudPrevious.size() > 270)
	{
		int numPoints = std::min(this->input.pointCloud.size(), this->input.pointCloudPrevious.size());

		//ROS_INFO("Points ...");
		for(int k = 0; k < numPoints - 2; k++)
		{
			PointT p0, p1, p2;
			p0 = getPointFromCurrentScan(k + 1);
			p1 = getPointFromPreviousScan(k + 1);
			p2 = getPointFromPreviousScan(k + 2);
			
			if(shouldThisTriangleBeDrawn(p0, p1, p2))
			{
				drawTriangle(p0, p1, p2);
			}

			p0 = getPointFromCurrentScan(k + 1);
			p1 = getPointFromCurrentScan(k + 2);
			p2 = getPointFromPreviousScan(k + 2);
			
			if(shouldThisTriangleBeDrawn(p0, p1, p2))
			{
				drawTriangle(p0, p1, p2);
			}
		}
	}
	else
	{
		//ROS_INFO("No points ...");
	}

	if(triangle_list.points.size() > 0)
	{
		char ns2[255];
		sprintf(ns2, " %d", 5);
		std::string name = "Testing";
		name.append(ns2);
		triangle_list.ns = name.c_str();
		this->markerPublisher.publish(triangle_list);
	}
}

void TriangleDrawer::publishPointsCloseToScanner()
{
	// Close points
	// ============

	sensor_msgs::PointCloud2 pointCloud;

	pcl::PointCloud<PointT> pCloud = this->input.pointCloud;
	pCloud.clear();
	PointT p;

	if (this->input.pointCloud.size())
	{
		//	Pre-Filter data
		for (int i = 0; i < this->input.pointCloud.size(); i++)
		{
			p = this->input.pointCloud.at(i);
			if(isPointWithinDesiredArea(p))
			{
				pCloud.push_back(p);
			}
		}
	}

	pcl::toROSMsg(pCloud, pointCloud);
	this->closePointsPublisher.publish(pointCloud);
}

bool TriangleDrawer::shouldThisTriangleBeDrawn(PointT p0, PointT p1, PointT p2)
{
	if(isPointWithinDesiredArea(p0) 
			&& isPointWithinDesiredArea(p1) 
			&& isPointWithinDesiredArea(p2)
			&& isPointsWithinACertainDistance(p0, p1, 0.2)
			&& isPointsWithinACertainDistance(p0, p2, 0.2)
			&& isPointsWithinACertainDistance(p1, p2, 0.2))
	{
		return true;
	}
	return false;
}

void TriangleDrawer::drawTriangle(PointT p0, PointT p1, PointT p2)
{
	addPointToTriangleList(p0);
	addPointToTriangleList(p1);
	addPointToTriangleList(p2);
	addColorToTriangleList(165 / 255., 32 / 255., 25 / 256.);
}

bool TriangleDrawer::isPointWithinDesiredArea(PointT p)
{
	double l = std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2) + std::sqrt(std::pow(p.z, 2)));

	// Is point out of a defined distance to scanner.
	if (minDist > l)
		return false;
	if(l > maxDist) 
		return false;

	// Is point behind scanner.
	if(p.x < 0)
		return false;
	return true;
}

bool TriangleDrawer::isPointsWithinACertainDistance(PointT a, PointT b, double thresholdDist)
{
	double dist = distanceBetweenPoints(a, b);
	if(dist < thresholdDist)
		return true;
	else
		return false;
}

double TriangleDrawer::distanceBetweenPoints(PointT a, PointT b)
{
	double dx = a.x - b.x;
	double dy = a.y - b.y;
	double dz = a.z - b.z;
	double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
	return dist;
}

PointT TriangleDrawer::getPointFromCurrentScan(int index)
{
	return this->input.pointCloud.at(index);
}

PointT TriangleDrawer::getPointFromPreviousScan(int index)
{
	return this->input.pointCloudPrevious.at(index);
}

void TriangleDrawer::addPointToTriangleList(PointT p)
{
	geometry_msgs::Point p0;
	p0.x = p.x;
	p0.y = p.y;
	p0.z = p.z;
	triangle_list.points.push_back(p0);
}

void TriangleDrawer::addColorToTriangleList(double r, double g, double b)
{
	std_msgs::ColorRGBA color;
	color.r = r;
	color.b = b;
	color.g = g;
	triangle_list.colors.push_back(color);
}

void TriangleDrawer::addPointFromCurrentScan(int index)
{
	PointT p = getPointFromCurrentScan(index);
	addPointToTriangleList(p);
}

void TriangleDrawer::addPointFromPreviousScan(int index)
{
	PointT p = getPointFromPreviousScan(index);
	addPointToTriangleList(p);
}

void TriangleDrawer::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
	try
	{

		//	Transform laser scan to point cloud in the scanners own frame
		sensor_msgs::PointCloud2 pointCloud;
		this->laserProjection.transformLaserScanToPointCloud("/base_link", *data, pointCloud, this->tfListener);
		input.pointCloudPrevious = input.pointCloud;
		pcl::fromROSMsg(pointCloud, this->input.pointCloud);
	}
	catch(...)
	{
	}
}

