/*
 * RowExtractorNode.cpp
 *
 *  Created on: Aug 1, 2013
 *      Author: kent
 */

#include "RowExtractorNode.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "RowExtractorNode");

	RowExtractorNode rowNode;
	rowNode.makeItSpin();

	return 0;
}

RowExtractorNode::RowExtractorNode()
{
	//	ROS node handler stuff
	this->nodeHandler = ros::NodeHandle("~");
	this->nodeHandler.param<int>("loopRate", this->loopRate, 100);

	//	Setup laser scanner --> point cloud stuff
	this->tfListener.setExtrapolationLimit(ros::Duration(0.1));

	//	Setup debug publisher
	this->pointCloudPublisher = this->nodeHandler.advertise<sensor_msgs::PointCloud2>("pointCloudTest", 10);

	//	Setup marker
	this->marker.header.frame_id = "/laser_link";
	this->marker.ns = "LineMarker";
	this->marker.id = 0;
	this->marker.type = visualization_msgs::Marker::CUBE;
	this->marker.action = visualization_msgs::Marker::ADD;
	this->marker.lifetime = ros::Duration();
	this->marker.color.r = 0.0;
	this->marker.color.g = 0.0;
	this->marker.color.b = 1.0;
	this->marker.color.a = 0.8;

	this->markerPublisher = this->nodeHandler.advertise<visualization_msgs::Marker>("line_marker_pub", 10);

	// Setup system output
	this->nodeHandler.param<std::string>("rowTopic", this->output.rowTopic, "rowTopic");
	this->output.rowPublisher = this->nodeHandler.advertise<msgs::row>(this->output.rowTopic, 10);

	//	Setup system input
	this->nodeHandler.param<std::string>("scanTopic", this->input.scanTopic, "/fmSensors/laser_msg");
	this->nodeHandler.param<std::string>("scanLink", this->input.scanLink, "/laser_link");
	this->input.scanSubscribe = this->nodeHandler.subscribe<sensor_msgs::LaserScan>(this->input.scanTopic, 10, &RowExtractorNode::laserScanCallback, this);
}

RowExtractorNode::~RowExtractorNode()
{
	// TODO Auto-generated destructor stub
}

void RowExtractorNode::makeItSpin()
{
	ros::Rate r(this->loopRate);

	while (ros::ok() && this->nodeHandler.ok())
	{
		//	Update event que
		ros::spinOnce();

		//	Debug
		this->preProcess();
		this->ransac();

		sensor_msgs::PointCloud2 pointCloud;
		pcl::toROSMsg(this->preProcessedCloud, pointCloud);
		this->pointCloudPublisher.publish(pointCloud);

		this->marker.header.stamp = ros::Time::now();
		this->markerPublisher.publish(this->marker);

		//	Publish row data
		this->output.rowPublisher.publish(this->output.rowMessage);

		//	Delay looping according to rate
		r.sleep();
	}
}

void RowExtractorNode::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
	//	Transform laser scan to point cloud in the scanners own frame
	sensor_msgs::PointCloud2 pointCloud;
	this->laserProjection.transformLaserScanToPointCloud(this->input.scanLink, *data, pointCloud, this->tfListener);

	pcl::fromROSMsg(pointCloud, this->input.pointCloud);
}

void RowExtractorNode::preProcess (void)
{
	//	Parameters
	double minDist = .5; //	[m]
	double maxDist = 5.0; //	[m]

	//	Variables
	pcl::PointCloud<PointT> pCloud = this->input.pointCloud;
	pCloud.clear();
	PointT p;
	double l;

	if (this->input.pointCloud.size())
	{
		//ROS_INFO("Cloud size: %d", this->input.pointCloud.size());

		//	Pre-Filter data
		for (int i = 0; i < this->input.pointCloud.size(); i++)
		{
			p = this->input.pointCloud.at(i);
			l = std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2));

			if (minDist < l && l < maxDist) pCloud.push_back(p);
		}

		//ROS_INFO("Filtered cloud: %d", pCloud.size());
	}
//	else
//	{
//		ROS_INFO("Possibly no cloud...");
//	}

	this->preProcessedCloud = pCloud; //this->input.pointCloud;
}

void RowExtractorNode::ransac (void)
{
	//	Seed randomizer
	srand(time(0));

	//	Parameters
	int numberOfTries = 20;
	int minNumberOfPoints = 50;
	double distanceThreshold = .2;

	//	Variables
	int i = 0, pointCounter, bestCount = 0;
	int randomPoints[2];
	PointT lookingAt[2];
	PointT pointsOfInterest[2];
	PointT directionVector, orthogonalVector, helpVector, projectionVector, lineVector;
	PointT p;
	double dotProduct, length, projectionLength;

	//	Algorithm
	if (this->preProcessedCloud.size())
	{
		while (i < numberOfTries)
		{
			//	Select two points in cloud randomly
			randomPoints[0] = rand() % this->preProcessedCloud.size();
			randomPoints[1] = rand() % this->preProcessedCloud.size();

			lookingAt[0] = this->preProcessedCloud.at(randomPoints[0]);
			lookingAt[1] = this->preProcessedCloud.at(randomPoints[1]);

			//	Calculate direction from two randomly selected points
			directionVector.x = lookingAt[1].x - lookingAt[0].x;
			directionVector.y = lookingAt[1].y - lookingAt[0].y;

			//	Calculate orthogonal vector on direction
			orthogonalVector.x = - directionVector.y;
			orthogonalVector.y = directionVector.x;

			//	Reset point counter
			pointCounter = 0;

			//	Check distance to line to each point in cloud
			for (int j = 0; j < this->preProcessedCloud.size(); j++)
			{
				//	Get point
				p = this->preProcessedCloud.at(j);

				//	Make help vector for projection
				helpVector.x = p.x - lookingAt[0].x;
				helpVector.y = p.y - lookingAt[0].y;

				//	Make projection of helpVector onto orthogonalVector
				dotProduct = orthogonalVector.x * helpVector.x + orthogonalVector.y * helpVector.y;
				length = std::sqrt(std::pow(orthogonalVector.x, 2) + std::pow(orthogonalVector.y, 2));

				projectionVector.x = dotProduct / std::pow(length, 2) * orthogonalVector.x;
				projectionVector.y = dotProduct / std::pow(length, 2) * orthogonalVector.y;

				//	Calculate length of projection and decide if it is an inlier
				projectionLength = std::sqrt(std::pow(projectionVector.x, 2) + std::pow(projectionVector.y, 2));

				if (projectionLength < distanceThreshold)
				{
					pointCounter++;
				}
			}

			if (pointCounter > bestCount)
			{
				bestCount = pointCounter;
				pointsOfInterest[0] = lookingAt[0];
				pointsOfInterest[1] = lookingAt[1];
			}

			//	Add one try
			i++;
		}
	}

	//	Output
	if (bestCount > minNumberOfPoints)
	{
		ROS_INFO("Found line with %d inliers...", bestCount);

		PointT line, center;
		line.x = pointsOfInterest[1].x - pointsOfInterest[0].x;
		line.y = pointsOfInterest[1].y - pointsOfInterest[0].y;
		center.x = pointsOfInterest[0].x + line.x / 2;
		center.y = pointsOfInterest[0].y + line.y / 2;

		double lineLength, lineOrientation;
		lineLength = std::sqrt(std::pow(line.x, 2) + std::pow(line.y, 2));
		lineOrientation = std::atan2(line.y, line.x);

		this->marker.pose.position.x = center.x;
		this->marker.pose.position.y = center.y;

		this->marker.pose.orientation = tf::createQuaternionMsgFromYaw(lineOrientation);

		this->marker.scale.x = lineLength;
		this->marker.scale.y = 2 * distanceThreshold;
	}
	else
	{
		ROS_INFO("No line was found within point cloud...");

		this->marker.pose.position.x = 0;
		this->marker.pose.position.y = 0;

		this->marker.scale.x = 0;
		this->marker.scale.y = 0;

	}
}
