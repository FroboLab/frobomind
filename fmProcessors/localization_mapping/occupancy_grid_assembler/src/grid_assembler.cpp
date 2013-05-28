/****************************************************************************
 # FroboMind grid_assembler.cpp
 # Copyright (c) 2013, Rudi Hansen <dimx@dimx.dk>
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


// .hpp stuff
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <occupancy_grid_utils/combine_grids.h>
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/server.h>
#include "../cfg/cpp/occupancy_grid_assembler/MyStuffConfig.h" 

class GridAssembler
{
  /*
   * When a new map is received, it is projected to a temporary map with the correct orientation. Afterwards it is added to the global data map.
   */
public:
  GridAssembler();
  void spin(void);

protected:
  // Typedef
  typedef boost::shared_ptr<nav_msgs::OccupancyGrid> GridPtr;

  // Methods
  void setupParameters(void);
  void onGridMsg(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void initMap(nav_msgs::MapMetaData info);
  void onSensorvalue(const std_msgs::Float64::ConstPtr& msg);

  // ROS
  ros::NodeHandle rosNode;
  ros::Subscriber inMapSubscriber;
  ros::Publisher outMapPublisher;
  ros::Publisher dMapPublisher1;
  std::string inMapSubscriberTopic;
  std::string outMapPublisherTopic;

  // Dynamic reconfigure
  void configcallback(occupancy_grid_assembler::MyStuffConfig &config, uint32_t level);

  // Map
  nav_msgs::OccupancyGrid dataMap;
  GridPtr combined_grid;
  GridPtr aligned_value;
  GridPtr aligned_overlap;
  GridPtr old_aligned_value;
  GridPtr old_aligned_overlap;
  double mapResolution;
  int mapInitialized;

  // Map parameters
  double gain_divide_enter_area_increase_;
  double gain_divide_enter_area_decrease_;
  double gain_divide_overlap_area_increase_;
  double gain_divide_overlap_area_decrease_;
  double gain_divide_left_area_increase_;
  double gain_divide_left_area_decrease_;
  int filter_kernel_size_;

};



// .cpp stuff
GridAssembler::GridAssembler() : rosNode("~"), combined_grid(new nav_msgs::OccupancyGrid())
{
  // Parameters
  setupParameters();
  
  // Topics
  inMapSubscriber = rosNode.subscribe(inMapSubscriberTopic, 100, &GridAssembler::onGridMsg, this);
  outMapPublisher = rosNode.advertise<nav_msgs::OccupancyGrid>(outMapPublisherTopic, 10);
  dMapPublisher1 = rosNode.advertise<nav_msgs::OccupancyGrid>("/fmKnowledge/map_difference", 10);

  // Init data
  mapInitialized = false;

  // Dynamic reconfigure
  dynamic_reconfigure::Server<occupancy_grid_assembler::MyStuffConfig> configserver;
  dynamic_reconfigure::Server<occupancy_grid_assembler::MyStuffConfig>::CallbackType f;
  f = boost::bind(&GridAssembler::configcallback, this, _1, _2);
  configserver.setCallback(f);
}
void GridAssembler::spin()
{
  ros::spin();
}

void GridAssembler::configcallback(occupancy_grid_assembler::MyStuffConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request:");
}
void GridAssembler::setupParameters(void)
{
  rosNode.param<std::string>("in_map_topic", this->inMapSubscriberTopic, "/not_configured/map_in");
  rosNode.param<std::string>("out_map_topic", this->outMapPublisherTopic, "/not_configured/map_out");
  rosNode.param<double>("mapResolution", this->mapResolution, 0.1);
  rosNode.param<double>("gain_divide_enter_area_increase", this->gain_divide_enter_area_increase_, 8);
  rosNode.param<double>("gain_divide_enter_area_decrease", this->gain_divide_enter_area_decrease_, 40);
  rosNode.param<double>("gain_divide_overlap_area_increase", this->gain_divide_overlap_area_increase_, 40);
  rosNode.param<double>("gain_divide_overlap_area_decrease", this->gain_divide_overlap_area_decrease_, 40);
  rosNode.param<double>("gain_divide_left_area_increase", this->gain_divide_left_area_increase_, 20);
  rosNode.param<double>("gain_divide_left_area_decrease", this->gain_divide_left_area_decrease_, 40);
  rosNode.param<int>("filter_kernel_size", this->filter_kernel_size_, 40);
}
void GridAssembler::initMap(nav_msgs::MapMetaData info)
{
  //combined_grid->info = info;
  combined_grid->info.resolution = mapResolution;
  combined_grid->header.frame_id = "/odom";
  combined_grid->info.height = 10;
  combined_grid->info.width = 10;
  combined_grid->info.origin.position.x = 0;
  combined_grid->info.origin.position.y = 0;
  combined_grid->info.origin.position.z = 0;
  combined_grid->info.origin.orientation.x = 0;
  combined_grid->info.origin.orientation.y = 0;
  combined_grid->info.origin.orientation.z = 1;
  combined_grid->info.origin.orientation.w = 0;
  combined_grid->data.resize(combined_grid->info.height*combined_grid->info.width);
  fill(combined_grid->data.begin(), combined_grid->data.end(), -1);
  mapInitialized = true;
}
void GridAssembler::onGridMsg(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  // Check for zero sized map
  if ((msg->info.width == 0) | (msg->info.height == 0))
    return;

  // Make aligened grid for received grid
  aligned_value = occupancy_grid_utils::getAlignedGrid(msg, mapResolution);
  aligned_overlap = occupancy_grid_utils::getAlignedGrid(msg, mapResolution);

  // Combine information from new grid to aligned grid
  //occupancy_grid_utils::floatingCombineToEmptyGrid(aligned_value, aligned_overlap, msg);
  occupancy_grid_utils::binaryCombineToEmptyGrid(aligned_value, aligned_overlap, msg);

  // If the maps is not initialized, we have no grids to combine
  if (!mapInitialized)
    {
      old_aligned_value = aligned_value;
      old_aligned_overlap = aligned_overlap;
      initMap(msg->info);
      return;
    }

  // Make a difference grids (Only newly covered area is updated)
  GridPtr difference_grid = occupancy_grid_utils::informationCombineAlignedGrids
    (
     aligned_value, 
     aligned_overlap, 
     old_aligned_value,
     old_aligned_overlap,
     gain_divide_enter_area_increase_,
     gain_divide_enter_area_decrease_,
     gain_divide_overlap_area_increase_,
     gain_divide_overlap_area_decrease_,
     gain_divide_left_area_increase_,
     gain_divide_left_area_decrease_
     );
  
  if (filter_kernel_size_ > 0)
    difference_grid = occupancy_grid_utils::averagePassGrid(difference_grid, filter_kernel_size_);

  difference_grid->header.frame_id = "/odom";
  dMapPublisher1.publish(difference_grid);
  
  // Combine grids
  occupancy_grid_utils::combineToGrid(combined_grid, difference_grid);

  // Publish map
  outMapPublisher.publish(combined_grid);
  
  // Keep a copy of last map
  old_aligned_value = aligned_value;
  old_aligned_overlap = aligned_overlap;
}

// -------

int main(int argc, char** argv)
{
  ros::init(argc, argv, "GridAssembler");

  GridAssembler gridAssembler;
  gridAssembler.spin();

  return 0;
}
