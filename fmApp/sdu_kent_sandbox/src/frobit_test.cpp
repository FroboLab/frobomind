#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>

int main (int argc, char** argv)
{
	ros::init(argc, argv, "Frobit_odom_test");

	ros::NodeHandle n("~");

	std::string twistTopic = "/fmSignals/cmd_vel";
	std::string deadTopic = "/fmSignals/deadman";

	ros::Publisher twistPub = n.advertise<geometry_msgs::TwistStamped>(twistTopic, 10);
	ros::Publisher deadPub = n.advertise<std_msgs::Bool>(deadTopic, 10);

	geometry_msgs::TwistStamped twist;
	std_msgs::Bool dead;

	ros::Time startTime;
	ros::Duration timeToRun(10.0), dt;
	ros::Rate loopRate(100);

	startTime = ros::Time::now();

	while (ros::ok() && n.ok())
	{
		ros::spinOnce();

		dt.fromNSec(ros::Time::now().toNSec() - startTime.toNSec());

		//	Setup msgs
		if (dt < timeToRun)
		{
			dead.data = true;
			twist.twist.linear.x = .5;
			twist.twist.angular.z = .0;
		}
		else
		{
			dead.data = false;
			twist.twist.linear.x = .0;
			twist.twist.angular.z = .0;
		}

		//	Publish
		deadPub.publish(dead);
		twist.header.stamp = ros::Time::now();
		twistPub.publish(twist);

		loopRate.sleep();
	}

	return 0;
}
