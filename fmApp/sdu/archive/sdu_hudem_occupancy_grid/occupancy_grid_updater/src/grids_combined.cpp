/****************************************************************************
 # FroboMind grids_combined.cpp
 # Copyright (c) 2012, Leon Bonde Larsen <leon@bondelarsen.dk>
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 #	* Redistributions of source code must retain the above copyright
 #  	notice, this list of conditions and the following disclaimer.
 #	* Redistributions in binary form must reproduce the above copyright
 #  	notice, this list of conditions and the following disclaimer in the
 #  	documentation and/or other materials provided with the distribution.
 #	* Neither the name FroboMind nor the
 #  	names of its contributors may be used to endorse or promote products
 #  	derived from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 # ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 # DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 # DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 # (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 # ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 # SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 #
 ****************************************************************************/
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <occupancy_grid_utils/combine_grids.h>

nav_msgs::OccupancyGrid initial_map;
nav_msgs::OccupancyGrid::Ptr grid_p;
std::vector<nav_msgs::OccupancyGrid::ConstPtr> global_list;
std::vector<nav_msgs::OccupancyGrid::ConstPtr> local_list;
ros::Publisher grid_pub;//, zero_pub;
nav_msgs::OccupancyGrid grid_msg;

void onGridMsg(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
//	local_list.push_back(msg);
//	if(local_list.size() > 1)
//	{
//		grid_p = occupancy_grid_utils::zeroCombineGrids(local_list);
//		grid_p = msg;
//		grid_p->header.frame_id = "/odom";
//		local_list.erase(local_list.begin());
		global_list.push_back(msg);
//		zero_pub.publish(grid_p);
//		if(local_list.size() > 1)
//			ROS_WARN("List has grown...");
//	}

}

void onTimer(const ros::TimerEvent&)
{
	if(global_list.size() > 1)
	{
		if (global_list.size() > 100)
		{
			grid_p = occupancy_grid_utils::minCombineGrids(global_list);
			grid_p->header.frame_id = "/odom";
			grid_pub.publish(grid_p);
			global_list.clear();
			global_list.push_back(grid_p);
		}
		else
		{
			grid_p = occupancy_grid_utils::minCombineGrids(global_list);
			grid_p->header.frame_id = "/odom";
			grid_pub.publish(grid_p);
		}

	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "combine_grids");
	ros::NodeHandle globalNodeHandler;
	grid_pub = globalNodeHandler.advertise<nav_msgs::OccupancyGrid>("/fmKnowledge/map_combined", 100);
//	zero_pub = globalNodeHandler.advertise<nav_msgs::OccupancyGrid>("/fmKnowledge/map_zeroed", 100);
	ros::Subscriber grid_sub = globalNodeHandler.subscribe("/fmKnowledge/map", 10, onGridMsg);
	ros::Timer timer = globalNodeHandler.createTimer(ros::Duration(1), onTimer);
	ros::spin();
	return 0;
}
