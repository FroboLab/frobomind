/*
 * TriangleDrawer.h
 *
 *  Created on: Aug 1, 2013
 *      Author: kent
 */

#ifndef ROWEXTRACTORNODE_H_
#define ROWEXTRACTORNODE_H_

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <msgs/row.h>
#include <visualization_msgs/Marker.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZI PointT;

class TriangleDrawer
{
	//	Node handler
	ros::NodeHandle nodeHandler;
	int loopRate;

	//	Laser scan --> Point cloud
	tf::TransformListener tfListener;
	laser_geometry::LaserProjection laserProjection;

	//	Debug
	bool debugActive;
	ros::Publisher pointCloudPublisher;
	ros::Publisher markerPublisher;
	ros::Publisher closePointsPublisher;

	//	System input
	struct
	{
		pcl::PointCloud<PointT> pointCloud;
		pcl::PointCloud<PointT> pointCloudPrevious;
		std::string scanTopic;
		std::string scanLink;
		ros::Subscriber scanSubscribe;
	} input;

	//	System output
	struct
	{
		msgs::row rowMessage;
		std::string rowTopic;
		ros::Publisher rowPublisher;
	} output;

	//	Callback
	void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& data);

	// 	Parameters
	double minDist;
	double maxDist;

	
	visualization_msgs::Marker triangle_list;
	void publishTriangleSurfaces(int counter);
	void publishPointsCloseToScanner();
	bool shouldThisTriangleBeDrawn(PointT p0, PointT p1, PointT p2);
	void drawTriangle(PointT p0, PointT p1, PointT p2);
	bool isPointWithinDesiredArea(PointT p);
	bool isPointsWithinACertainDistance(PointT a, PointT b, double thresholdDist);
	double distanceBetweenPoints(PointT a, PointT b);
	PointT getPointFromCurrentScan(int index);
	PointT getPointFromPreviousScan(int index);
	void addPointToTriangleList(PointT p);
	void addColorToTriangleList(double r, double g, double b);
	void addPointFromCurrentScan(int index);
	void addPointFromPreviousScan(int index);

public:
	TriangleDrawer();
	virtual ~TriangleDrawer();

	void makeItSpin (void);
};

#endif /* ROWEXTRACTORNODE_H_ */
