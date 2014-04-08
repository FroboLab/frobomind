/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */


/**
 * \file
 *
 * Implementation for combine_grids.h
 *
 * \author Bhaskara Marthi
 */

#include <occupancy_grid_utils/combine_grids.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <ros/assert.h>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <set>
#include "gcc_version.h"


namespace occupancy_grid_utils
{

namespace nm=nav_msgs;
namespace gm=geometry_msgs;

using boost::bind;
using boost::ref;
using std::vector;
using std::multiset;
using std::set;
using std::min;
using std::max;

typedef boost::shared_ptr<nm::OccupancyGrid> GridPtr;
typedef boost::shared_ptr<nm::OccupancyGrid const> GridConstPtr;



inline Cell point32Cell (const nm::MapMetaData& info, const gm::Point32& p)
{
  gm::Point pt;
  pt.x = p.x;
  pt.y = p.y;
  pt.z = p.z;
  return pointCell(info, pt);
}

// Does this cell contain a vertex of this polygon?
inline bool containsVertex (const nm::MapMetaData& info, const Cell& c, const gm::Polygon& poly)
{
  BOOST_FOREACH (const gm::Point32& p, poly.points) {
    if (point32Cell(info, p)==c)
      return true;
  }
  return false;    
}


// Do the two cells (on different grids) intersect?
inline bool cellsIntersect (const nm::MapMetaData& info1, const Cell& c1, const nm::MapMetaData& info2, const Cell& c2)
{
  const gm::Polygon p1=cellPolygon(info1, c1);
  const gm::Polygon p2=cellPolygon(info2, c2);
  return containsVertex(info1, c1, p2) || containsVertex(info2, c2, p1);
}

inline gm::Polygon expandPolygon(const gm::Polygon& p, const double r)
{
  double sx=0;
  double sy=0;
  double sz=0;
  const size_t n = p.points.size();
  for (unsigned i=0; i<n; i++) {
    sx += p.points[i].x;
    sy += p.points[i].y;
    sz += p.points[i].z;
  }
  sx /= n;
  sy /= n;
  sz /= n;
  gm::Polygon p2;
  p2.points.resize(n);
  for (unsigned i=0; i<n; i++) {
    p2.points[i].x = sx + r*(p.points[i].x-sx);
    p2.points[i].y = sy + r*(p.points[i].y-sy);
    p2.points[i].z = sz + r*(p.points[i].z-sz);
  }
  return p2;
}

// Return set of intersecting cells in grid info, of cell cell2 in info2
set<Cell> intersectingCells (const nm::MapMetaData& info, const nm::MapMetaData& info2, const Cell& cell2)
{
  // The expansion is to avoid weird effects due to rounding when intersecting parallel grids
  const gm::Polygon poly=expandPolygon(cellPolygon(info2, cell2), 1.0001);

  // Figure out the candidates
  vector<Cell> corners(4);
  transform(poly.points.begin(), poly.points.end(), corners.begin(), 
            bind(point32Cell, ref(info), _1));
  const coord_t min_x=min(corners[0].x, min(corners[1].x, min(corners[2].x, corners[3].x)));
  const coord_t min_y=min(corners[0].y, min(corners[1].y, min(corners[2].y, corners[3].y)));
  const coord_t max_x=max(corners[0].x, max(corners[1].x, max(corners[2].x, corners[3].x)));
  const coord_t max_y=max(corners[0].y, max(corners[1].y, max(corners[2].y, corners[3].y)));
  
  set<Cell> cells;
  for (coord_t x=min_x; x<=max_x; x++) {
    for (coord_t y=min_y; y<=max_y; y++) {
      const Cell candidate(x, y);
      if (withinBounds(info, candidate) &&
          cellsIntersect(info, candidate, info2, cell2))
        cells.insert(candidate);
    }
  }

  return cells;
}

double minX (const nm::MapMetaData& info)
{
  const gm::Polygon p=gridPolygon(info);
  return min(p.points[0].x, min(p.points[1].x, min(p.points[2].x, p.points[3].x)));
}

double maxX (const nm::MapMetaData& info)
{
  const gm::Polygon p=gridPolygon(info);
  return max(p.points[0].x, max(p.points[1].x, max(p.points[2].x, p.points[3].x)));
}

double minY (const nm::MapMetaData& info)
{
  const gm::Polygon p=gridPolygon(info);
  return min(p.points[0].y, min(p.points[1].y, min(p.points[2].y, p.points[3].y)));
}

double maxY (const nm::MapMetaData& info)
{
  const gm::Polygon p=gridPolygon(info);
  return max(p.points[0].y, max(p.points[1].y, max(p.points[2].y, p.points[3].y)));
}

gm::Pose transformPose (const tf::Pose trans, const gm::Pose p)
{
  tf::Pose pose;
  tf::poseMsgToTF(p, pose);
  gm::Pose transformed;
  tf::poseTFToMsg(trans*pose, transformed);
  return transformed;
}

// Get the dimensions of a combined grid
nm::MapMetaData getCombinedGridInfo (const vector<GridConstPtr>& grids, const double resolution)
{
  ROS_ASSERT (grids.size() > 0);
  nm::MapMetaData info;
  info.resolution = resolution;
  tf::Pose trans;
  tf::poseMsgToTF(grids[0]->info.origin, trans);
  

#ifdef GRID_UTILS_GCC_46
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
#endif

  boost::optional<double> min_x, max_x, min_y, max_y;
  BOOST_FOREACH (const GridConstPtr& g, grids) {
    nm::MapMetaData grid_info = g->info;
    grid_info.origin = transformPose(trans.inverse(), g->info.origin);
    //if (!(min_x && *min_x < minX(grid_info)))
      min_x = minX(grid_info);
      //if (!(min_y && *min_y < minY(grid_info)))
      min_y = minY(grid_info);
      //if (!(max_x && *max_x > maxX(grid_info)))
      max_x = maxX(grid_info);
      //if (!(max_y && *max_y > maxY(grid_info)))
      max_y = maxY(grid_info);
  }
  
#ifdef GRID_UTILS_GCC_46
#pragma GCC diagnostic pop
#endif

  const double dx = *max_x - *min_x;
  const double dy = *max_y - *min_y;
  ROS_ASSERT ((dx > 0) && (dy > 0));
  gm::Pose pose_in_grid_frame;
  pose_in_grid_frame.position.x = *min_x;
  pose_in_grid_frame.position.y = *min_y;
  pose_in_grid_frame.orientation.w = 1.0;
  info.origin = transformPose(trans, pose_in_grid_frame);
  info.height = ceil(dy/info.resolution);
  info.width = ceil(dx/info.resolution);

  return info;
}
nm::MapMetaData getAlignedGridInfo(const nav_msgs::OccupancyGrid::ConstPtr& toBeAlignedGrid, const double resolution)
{
  nm::MapMetaData info;

  const double roundoff_fix = 0.0001;

  double max_x = maxX(toBeAlignedGrid->info);
  double min_x = minX(toBeAlignedGrid->info);
  double max_y = maxY(toBeAlignedGrid->info);
  double min_y = minY(toBeAlignedGrid->info);

  // Fit to aligned grid
  int cells_max_x = ceil(max_x/resolution);
  int cells_max_y = ceil(max_y/resolution);
  int cells_min_x = floor((min_x+roundoff_fix)/resolution); // fixme: roundoff problem
  int cells_min_y = floor((min_y+roundoff_fix)/resolution);

  int cells_h = cells_max_y - cells_min_y;
  int cells_w = cells_max_x - cells_min_x;

  info.width = cells_w;
  info.height = cells_h;
  info.origin.position.x = cells_max_x * resolution;
  info.origin.position.y = cells_max_y * resolution;
  info.origin.position.z = 0;
  info.resolution = resolution;
  info.origin.orientation.x = 0;
  info.origin.orientation.y = 0;
  info.origin.orientation.z = 1;
  info.origin.orientation.w = 0;

  return info;
}
nm::MapMetaData getMaxGridInfo(const nm::MapMetaData info1, const nm::MapMetaData info2)
{
  /*
   * Assuming orientation is the same on both grids
   */
  ROS_ASSERT(info1.resolution == info2.resolution);
  const double roundoff_fix = 0.0001;
  nm::MapMetaData info;

  info.resolution = info1.resolution;

  double max_x = max( maxX(info1) , maxX(info2));
  double min_x = min( minX(info1) , minX(info2));
  double max_y = max( maxY(info1) , maxY(info2));
  double min_y = min( minY(info1) , minY(info2));

  // Fit to aligned grid
  int cells_max_x = ceil(max_x/info.resolution);
  int cells_max_y = ceil(max_y/info.resolution);
  int cells_min_x = floor((min_x+roundoff_fix)/info.resolution); // fixme: roundoff problem
  int cells_min_y = floor((min_y+roundoff_fix)/info.resolution);

  int cells_h = cells_max_y - cells_min_y;
  int cells_w = cells_max_x - cells_min_x;

  ROS_INFO("Combined grid size: %i %i - %i %i -- %i %i", cells_max_x, cells_max_y, cells_min_x, cells_min_y, cells_w, cells_h);

  info.width = cells_w;
  info.height = cells_h;
  info.origin.position.x = cells_max_x * info.resolution;
  info.origin.position.y = cells_max_y * info.resolution;
  info.origin.position.z = 0;
  info.origin.orientation.x = 0;
  info.origin.orientation.y = 0;
  info.origin.orientation.z = 1;
  info.origin.orientation.w = 0;

  return info;
}

GridPtr getAlignedGrid (const nav_msgs::OccupancyGrid::ConstPtr& toBeAlignedGrid, const double resolution)
{
  // Create new grid with new dimmension
  GridPtr resizedGrid(new nm::OccupancyGrid());
  resizedGrid->header.frame_id = toBeAlignedGrid->header.frame_id;
  resizedGrid->info = getAlignedGridInfo(toBeAlignedGrid, resolution);
  resizedGrid->data.resize(resizedGrid->info.width * resizedGrid->info.height);
  fill(resizedGrid->data.begin(),resizedGrid->data.end(), -1);

  return resizedGrid;
}

void resizeToAligedGrid(GridPtr& targetGrid, const nav_msgs::OccupancyGrid::ConstPtr& applyGrid)
{
  /*
    Warning: Some assumptions about grid orientation has been made to simplify the code
  */
  const double roundoff_fix = 0.0001;

  double old_max_x, old_min_x, old_max_y, old_min_y;
  double max_x = old_max_x = targetGrid->info.origin.position.x;
  double min_x = old_min_x = max_x - (targetGrid->info.width * targetGrid->info.resolution);
  double max_y = old_max_y = targetGrid->info.origin.position.y;
  double min_y = old_min_y = max_y - (targetGrid->info.height * targetGrid->info.resolution);
  int do_grid_resize = 0;

  // New dimmensions
  if (min_x > minX(applyGrid->info))
    {
      min_x = minX(applyGrid->info);
      do_grid_resize = 1;
    }
  if (min_y > minY(applyGrid->info))
    {
      min_y = minY(applyGrid->info);
      do_grid_resize = 1;
    }
  if (max_x < maxX(applyGrid->info))
    {
      max_x = maxX(applyGrid->info);
      do_grid_resize = 1;
    }
  if (max_y < maxY(applyGrid->info))
    {
      max_y = maxY(applyGrid->info);
      do_grid_resize = 1;
    }

  if (!do_grid_resize)
    return;

  // Fit to grid resolution
  int cells_max_x = ceil(max_x/targetGrid->info.resolution);
  int cells_max_y = ceil(max_y/targetGrid->info.resolution);
  int cells_min_x = floor((min_x+roundoff_fix)/targetGrid->info.resolution); // fixme: roundoff problem
  int cells_min_y = floor((min_y+roundoff_fix)/targetGrid->info.resolution);
  int old_cells_max_x = ceil(old_max_x/targetGrid->info.resolution);
  int old_cells_max_y = ceil(old_max_y/targetGrid->info.resolution);
  int old_cells_min_x = floor((old_min_x+roundoff_fix)/targetGrid->info.resolution); // fixme: roundoff problem
  int old_cells_min_y = floor((old_min_y+roundoff_fix)/targetGrid->info.resolution);

  //ROS_INFO("Fittet grid size %i, %i, %i, %i", cells_min_x, cells_max_x, cells_min_y, cells_max_y);
  int cells_h = cells_max_y - cells_min_y;
  int cells_w = cells_max_x - cells_min_x;
  int old_cells_h = old_cells_max_y - old_cells_min_y;
  int old_cells_w = old_cells_max_x - old_cells_min_x;

  // new values
  max_x = targetGrid->info.resolution * cells_max_x;
  max_y = targetGrid->info.resolution * cells_max_y;
  min_x = targetGrid->info.resolution * cells_min_x;
  min_y = targetGrid->info.resolution * cells_min_y;

  //ROS_INFO("Fittet map size %f, %f, %f, %f", min_x, max_x, min_y, max_y);

  // Create new grid with new dimmension
  GridPtr resizedGrid(new nm::OccupancyGrid());
  resizedGrid->header.frame_id = targetGrid->header.frame_id;
  resizedGrid->info.width = cells_w;
  resizedGrid->info.height = cells_h;
  resizedGrid->info.resolution = targetGrid->info.resolution;
  resizedGrid->info.origin.position.x = max_x;
  resizedGrid->info.origin.position.y = max_y;
  resizedGrid->info.origin.position.z = 0;
  resizedGrid->info.origin.orientation.x = 0;
  resizedGrid->info.origin.orientation.y = 0;
  resizedGrid->info.origin.orientation.z = 1;
  resizedGrid->info.origin.orientation.w = 0;
  resizedGrid->data.resize(resizedGrid->info.width * resizedGrid->info.height);
  fill(resizedGrid->data.begin(),resizedGrid->data.end(), -1);

  // Find offset from old grid
  int cells_offset_x;
  int cells_offset_y;

  cells_offset_y = cells_max_y - (old_cells_min_y + targetGrid->info.height);
  cells_offset_x = cells_max_x - (old_cells_min_x + targetGrid->info.width);


  // Copy old information to new grid

  for (int x = 0; x < targetGrid->info.width; x++)
    {
      for (int y = 0; y < targetGrid->info.height; y++)
	{
	  resizedGrid->data[(cells_offset_y + y)*resizedGrid->info.width+cells_offset_x + x] = targetGrid->data[y * targetGrid->info.width+x];
	}
    }

  // Switch to new grid
  targetGrid = resizedGrid;
}

Cell aligenedGridOffset(const nm::MapMetaData main_info, const nm::MapMetaData secondary_info)
{
  const double roundoff_fix = 0.0001;
  Cell offset;
  //offset.x = floor((main_info.origin.position.x - secondary_info.origin.position.x) / main_info.resolution);
  //offset.y = floor((main_info.origin.position.y - secondary_info.origin.position.y) / main_info.resolution);

  double max_x = main_info.origin.position.x;
  double max_y = main_info.origin.position.y;
  int cells_max_x = ceil(max_x/main_info.resolution);
  int cells_max_y = ceil(max_y/main_info.resolution);

  double sec_min_x = minX(secondary_info);
  double sec_min_y = minY(secondary_info);

  int sec_cells_min_x = floor((sec_min_x+roundoff_fix)/secondary_info.resolution); // fixme: roundoff problem
  int sec_cells_min_y = floor((sec_min_y+roundoff_fix)/secondary_info.resolution);

  offset.y = cells_max_y - (sec_cells_min_y + secondary_info.height);
  offset.x = cells_max_x - (sec_cells_min_x + secondary_info.width);
  return offset;
}

// Main function
GridPtr combineGrids (const vector<GridConstPtr>& grids, const double resolution)
{
  GridPtr combined_grid(new nm::OccupancyGrid());
  combined_grid->info = getCombinedGridInfo(grids, resolution);
  combined_grid->data.resize(combined_grid->info.width*combined_grid->info.height);
  fill(combined_grid->data.begin(), combined_grid->data.end(), -1);
  ROS_DEBUG_NAMED ("combine_grids", "Combining %zu grids", grids.size());

  BOOST_FOREACH (const GridConstPtr& grid, grids) {
    for (coord_t x=0; x<(int)grid->info.width; x++) {
      for (coord_t y=0; y<(int)grid->info.height; y++) {
        const Cell cell(x, y);
        const signed char value=grid->data[cellIndex(grid->info, cell)];

        // Only proceed if the value is not unknown 
        if ((value>=0) && (value<=100)) {
          BOOST_FOREACH (const Cell& intersecting_cell, 
                         intersectingCells(combined_grid->info, grid->info, cell)) {
            const index_t ind = cellIndex(combined_grid->info, intersecting_cell);
            combined_grid->data[ind] = max(combined_grid->data[ind], value);
          }
        }
      }
    }
  }

  ROS_DEBUG_NAMED ("combine_grids", "Done combining grids");
  return combined_grid;
}


GridPtr combineGrids (const vector<GridConstPtr>& grids)
{
  ROS_ASSERT (grids.size()>0);
  return combineGrids(grids, grids[0]->info.resolution);
}

// Main function
GridPtr minCombineGrids (const vector<GridConstPtr>& grids, const double resolution)
{
  GridPtr combined_grid(new nm::OccupancyGrid());
  combined_grid->info = getCombinedGridInfo(grids, resolution);
  combined_grid->data.resize(combined_grid->info.width*combined_grid->info.height);
  fill(combined_grid->data.begin(), combined_grid->data.end(), -1);
  ROS_DEBUG_NAMED ("combine_grids", "Combining %zu grids", grids.size());

  BOOST_FOREACH (const GridConstPtr& grid, grids) {
    for (coord_t x=0; x<(int)grid->info.width; x++) {
      for (coord_t y=0; y<(int)grid->info.height; y++) {
        const Cell cell(x, y);
        const signed char value=grid->data[cellIndex(grid->info, cell)];

        // Only proceed if the value is not unknown
        if ((value>=0) && (value<=100)) {
          BOOST_FOREACH (const Cell& intersecting_cell,
                         intersectingCells(combined_grid->info, grid->info, cell)) {
            const index_t ind = cellIndex(combined_grid->info, intersecting_cell);
            if(combined_grid->data[ind] != -1)
            	combined_grid->data[ind] = min(combined_grid->data[ind], value);
            else
            	combined_grid->data[ind] = value;

            if(combined_grid->data[ind] == -1)
            	combined_grid->data[ind] = 50;

            if(value > 50)
            	combined_grid->data[ind] += (value - 50)*0.5;
            else
            	combined_grid->data[ind] -= (50 - value)*0.5;

            if(combined_grid->data[ind] > 100)
            	combined_grid->data[ind] = 100;
            if(combined_grid->data[ind] < 0)
            	combined_grid->data[ind] = 0;

          }
        }
      }
    }
  }

  ROS_DEBUG_NAMED ("combine_grids", "Done combining grids");
  return combined_grid;
}


GridPtr minCombineGrids (const vector<GridConstPtr>& grids)
{
  ROS_ASSERT (grids.size()>0);
  return minCombineGrids(grids, grids[0]->info.resolution);
}

void combineToGrid( GridPtr& targetGrid, const nav_msgs::OccupancyGrid::ConstPtr& applyGrid )
{
  resizeToAligedGrid(targetGrid, applyGrid);

  // Place input map on main map
  for (coord_t x=0; x<(int)applyGrid->info.width; x++) {
    for (coord_t y=0; y<(int)applyGrid->info.height; y++) {
      const Cell cell(x, y);
      const signed char value=applyGrid->data[cellIndex(applyGrid->info, cell)];
      // Only proceed if the value is not unknown
      if ((value>=0) && (value<=100))
	{
	  BOOST_FOREACH (const Cell& intersecting_cell,
			 intersectingCells(targetGrid->info, applyGrid->info, cell))
	    {
	      const index_t ind = cellIndex(targetGrid->info, intersecting_cell);
	      int newvalue = targetGrid->data[ind];

	      // Init cell if undefined
	      if(newvalue == -1)
            	newvalue = 50;

	      //targetGrid->data[ind] = value;


	      if(value > 50)
            	newvalue += (value - 50); ///20;
	      else
            	newvalue -= (50 - value); ///20;


	      if(newvalue > 100)
		{
		  newvalue = 100;
		  //ROS_INFO("OVER CUTT!");
		}
	      if(newvalue < 0)
		{
		  newvalue = 0;
		  //ROS_INFO("UNDER CUT!");
		}
	      targetGrid->data[ind] = newvalue;
	    }
	}
    }
  }

  return;
  // Data needed for combinedGridInfo.
  std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids;
  grids.push_back(targetGrid);
  grids.push_back(applyGrid);

  targetGrid = combineGrids(grids);

  return;
  /*
  // Find new size on main grid
  nav_msgs::MapMetaData newGridSize = occupancy_grid_utils::getCombinedGridInfo(grids, targetGrid->info.resolution);
  ROS_INFO("New grid information generated");

  // Resize if needed
  //if (newGridSize.equals(targetGrid->info))
    {
      killme++;
      GridPtr newGrid(new nm::OccupancyGrid());
      newGrid->info = newGridSize;
      newGrid->data.resize(newGridSize.width*newGridSize.height);
      fill(targetGrid->data.begin(), targetGrid->data.end(), -1);

      // Avoid infinite recursive calls.
      ROS_ASSERT(killme > 1);

      // Recursice call to copy data to new grid.
      //combineToGrid(newGrid, targetGrid);


      targetGrid = newGrid;
    }
  killme = 0;


  */
}

// Main function
GridPtr zeroCombineGrids (const vector<GridConstPtr>& grids, const double resolution)
{
  GridPtr combined_grid(new nm::OccupancyGrid());
  combined_grid->info = getCombinedGridInfo(grids, resolution);
  combined_grid->data.resize(combined_grid->info.width*combined_grid->info.height);
  fill(combined_grid->data.begin(), combined_grid->data.end(), -1);
  ROS_DEBUG_NAMED ("combine_grids", "Combining %zu grids", grids.size());

  BOOST_FOREACH (const GridConstPtr& grid, grids)
  {
	  //make list
	   vector<GridConstPtr> list;

	   //push combined
	   list.push_back(combined_grid);

	   //push grid
	   list.push_back(grid);

	   //combine list
	   combined_grid = combineGrids(list); //TODO:memory leak?
  }
  BOOST_FOREACH (const GridConstPtr& grid, grids) {
    for (coord_t x=0; x<(int)grid->info.width; x++) {
      for (coord_t y=0; y<(int)grid->info.height; y++) {
        const Cell cell(x, y);
        const signed char value=grid->data[cellIndex(grid->info, cell)];

        // Only proceed if the value is not unknown
        if ((value>=0) && (value<=100)) {
          BOOST_FOREACH (const Cell& intersecting_cell,
                         intersectingCells(combined_grid->info, grid->info, cell)) {
            const index_t ind = cellIndex(combined_grid->info, intersecting_cell);
            	combined_grid->data[ind] = -1;
          }
        }
      }
    }
  }

  return combined_grid;
}

GridPtr zeroCombineGrids (const vector<GridConstPtr>& grids)
{
  ROS_ASSERT (grids.size()>0);
  return zeroCombineGrids(grids, grids[0]->info.resolution);
}


void binaryOverlapInformation(
			      double& overlap_area, // Due to compatibility with overlapInformation method
			      double& combined_value,
			      const nav_msgs::OccupancyGrid::ConstPtr& main_grid,
			      const Cell& main_grid_cell,
			      const nav_msgs::OccupancyGrid::ConstPtr& overlap_grid
			)
{
  // Find a starting cell on grid
  geometry_msgs::Point centerpoint = cellCenter(main_grid->info, main_grid_cell); // World coordinate

  // Reset values
  overlap_area = 0;
  combined_value = 0;

  // Find a corrospoding cell on the overlap grid
  Cell init_cell;
  init_cell = pointCell (overlap_grid->info, centerpoint);

  // Extend field of search
  coord_t min_x = init_cell.x-1;
  coord_t min_y = init_cell.y-1;
  coord_t max_x = init_cell.x+1;
  coord_t max_y = init_cell.y+1;

  // "Out of grid" checks
  if (min_x < 0) min_x = 0;
  if (min_y < 0) min_y = 0;
  if (max_x >= overlap_grid->info.width) max_x = overlap_grid->info.width -1;
  if (max_y >= overlap_grid->info.height) max_y = overlap_grid->info.height -1;

  for (coord_t cx=min_x; cx <= max_x; cx++)
    {
      for (coord_t cy=min_y; cy <= max_y; cy++)
	{
	  Cell intersecting_cell = Cell(cx, cy);
	  const index_t local_ind = cellIndex(overlap_grid->info, intersecting_cell);

	  // Only find overlap if cell contains data
	  if (overlap_grid->data[local_ind] != -1)
	    {

	      // Collect intersection information
	      if (cellsIntersect (main_grid->info, main_grid_cell, overlap_grid->info, intersecting_cell))
		{
		  overlap_area++;
		  combined_value += overlap_grid->data[local_ind];
		}
	    }
	}
    }

  // Normalize return values
  if (overlap_area > 1)
    {
      combined_value /= overlap_area; // return average value
      overlap_area = 1;
    }
}


void binaryCombineToEmptyGrid(GridPtr& combined_grid, GridPtr& overlap_grid, const nav_msgs::OccupancyGrid::ConstPtr& combine_from)
{
  // Assuming there will be space on the new grid..
  for (coord_t x=0; x<(int)combined_grid->info.width; x++)
    {
      for (coord_t y=0; y<(int)combined_grid->info.height; y++)
	{
	  const Cell cell(x, y); // Coordinate in new grid
	  const index_t combined_ind = cellIndex(combined_grid->info, cell);
	  double overlap_area = 0;
	  double cell_value = 0;

	  // Get overlapping information
	  binaryOverlapInformation(overlap_area,cell_value, combined_grid, cell, combine_from);

	  overlap_grid->data[combined_ind] = overlap_area * 100;
	  combined_grid->data[combined_ind] = cell_value;
	}
    }
}


GridPtr informationCombineAlignedGrids
(
 const nav_msgs::OccupancyGrid::ConstPtr& primary,
 const nav_msgs::OccupancyGrid::ConstPtr& primary_overlap,
 const nav_msgs::OccupancyGrid::ConstPtr& secondary,
 const nav_msgs::OccupancyGrid::ConstPtr& secondary_overlap,
 const double gain_divide_enter_area_increase,
 const double gain_divide_enter_area_decrease,
 const double gain_divide_overlap_area_increase,
 const double gain_divide_overlap_area_decrease,
 const double gain_divide_left_area_increase,
 const double gain_divide_left_area_decrease
 )
{
  // Get new dimmensions
  GridPtr combined_grid(new nm::OccupancyGrid());
  combined_grid->info = getMaxGridInfo(primary->info, secondary->info);
  combined_grid->header.frame_id = "/odom";
  combined_grid->data.resize(combined_grid->info.width*combined_grid->info.height);
  fill(combined_grid->data.begin(), combined_grid->data.end(), -1);

  // Find offset on primary
  Cell primary_offset = aligenedGridOffset(combined_grid->info, primary->info);
  Cell primary_endpoint;
  primary_endpoint.x = primary_offset.x + primary->info.width;
  primary_endpoint.y = primary_offset.y + primary->info.height;

  // Find offset on secondary
  Cell secondary_offset = aligenedGridOffset(combined_grid->info, secondary->info);
  Cell secondary_endpoint;
  secondary_endpoint.x = secondary_offset.x + secondary->info.width;
  secondary_endpoint.y = secondary_offset.y + secondary->info.height;

  // Find primary value
  // Assume max value at midpoint of grid
  int primary_max_value;
  {
    const index_t primary_ind= cellIndex(
					  primary->info,
					  Cell(
					       primary->info.width / 2,
					       primary->info.height / 2
					       )
					  );
    primary_max_value = primary->data[primary_ind];
  }

  // Find secondary value
  // Assume max value at midpoint of grid
  int secondary_max_value;
  {
    const index_t secondary_ind= cellIndex(
					  secondary->info,
					  Cell(
					       secondary->info.width / 2,
					       secondary->info.height / 2
					       )
					  );
    secondary_max_value = secondary->data[secondary_ind];
  }
  int value_change = primary_max_value - secondary_max_value;

  for (int x = 0; x < combined_grid->info.width; x++)
    {
      for (int y = 0; y < combined_grid->info.height; y++)
	{
	  const index_t combined_ind= cellIndex(combined_grid->info, Cell(x, y));
	  // Default values
	  int primary_value = 50; // 50 +/- 50
	  int primary_overlaps = 0; // [0-100]
	  int primary_overlaps_pct = 0;
	  int secondary_value = 50;
	  int secondary_overlaps = 0;
	  int combined_overlaps = 0;

	  try
	    {
	      const index_t primary_ind= cellIndex(primary->info, Cell(x-primary_offset.x, y-primary_offset.y));
	      primary_value = primary->data[primary_ind];
	      primary_overlaps = primary_overlap->data[primary_ind];
	    }
	  catch (CellOutOfBoundsException e)
	    {
	      //
	    }

	  try
	    {
	      const index_t secondary_ind= cellIndex(secondary->info, Cell(x-secondary_offset.x, y-secondary_offset.y));
	      secondary_value = secondary->data[secondary_ind];
	      secondary_overlaps = secondary_overlap->data[secondary_ind];
	    }
	  catch (CellOutOfBoundsException e)
	    {
	      //
	    }

	  int overlap_high = 60;
	  int overlap_low = 20;

	  if (primary_overlaps < 5 && secondary_overlaps < 5) // No information about cell
	    combined_grid->data[combined_ind] = -1; // -1
	  /*
	  else if (std::abs(secondary_overlaps - primary_overlaps) < 5) // Equal overlap
	    {
	      combined_grid->data[combined_ind] = 50;
	    }
	  */
	  else if (
		(primary_overlaps > 60) &&
		(secondary_overlaps < 20)
		) // Area entered
	    {
	      /*
	      if (value_change > 0) // Increasing value
		//combined_grid->data[combined_ind] = 50 + (primary_value - secondary_value)/2; //value_change;
		combined_grid->data[combined_ind] = 50 + (primary_value - 50)/gain_divide_enter_area_increase; //value_change;
	      else // Decreasing value
		combined_grid->data[combined_ind] = 50 + (primary_value - 50)/gain_divide_enter_area_decrease; //
	      */
	      if (value_change > 0) // Increasing value
		{
		  if (gain_divide_enter_area_increase > 0)
		    combined_grid->data[combined_ind] = 50 + (value_change)/gain_divide_enter_area_increase; //value_change;
		}
	      else // Decreasing value
		{
		  if (gain_divide_enter_area_decrease > 0)
		    combined_grid->data[combined_ind] = 50 + (value_change)/gain_divide_enter_area_decrease;
		}
	    }
	  else if (primary_overlaps > 75) // Covered by primary map
	    {
	      if (value_change > 0)
		{
		  if (gain_divide_overlap_area_increase > 0)
		    combined_grid->data[combined_ind] = 50 + (primary_value)/gain_divide_overlap_area_increase;
		}
	      else
		{
		  if (gain_divide_overlap_area_decrease > 0)
		    combined_grid->data[combined_ind] = 50 + (primary_value)/gain_divide_overlap_area_decrease;
		}
	    }
	    /*
	  else if (std::abs(secondary_overlaps - primary_overlaps) < 5) // Equal overlap
	    {
	      if (value_change > 0)
		combined_grid->data[combined_ind] = 50 + (primary_value - secondary_value)/gain_divide_overlap_area_increase;
	      else
		combined_grid->data[combined_ind] = 50 + (primary_value - secondary_value)/gain_divide_overlap_area_decrease;
	    }
	    */
	  else if (primary_overlaps < secondary_overlaps) // Area left
	    {
	      if (value_change <= 0) // Decresing value
		{
		  if (gain_divide_left_area_decrease > 0)
		  combined_grid->data[combined_ind] = 50 + (secondary_value - 50)/gain_divide_left_area_decrease; //value_change;
		}
	      else // Increasing value
		{
		  if (gain_divide_left_area_increase > 0)
		    combined_grid->data[combined_ind] = 50 + (secondary_value - 50)/gain_divide_left_area_increase; // + (primary_value - secondary_value)/2;
		}
	    }
	  else
	    combined_grid->data[combined_ind] = 50;
	}
    }

  return combined_grid;
}


GridPtr averagePassGrid (const nav_msgs::OccupancyGrid::ConstPtr& grid, int kernelsize)
{
  const int kernelmin = kernelsize * -1;
  const int kernelmax = kernelsize;

  GridPtr filtered(new nm::OccupancyGrid());
  filtered->info = grid->info; // Resize to fit new grid data
  filtered->data.resize(filtered->info.width*filtered->info.height);

  fill(filtered->data.begin(), filtered->data.end(), -1);


  for (int x = 0; x < filtered->info.width; x++)
    {
      for (int y = 0; y < filtered->info.height; y++)
	{
	  const Cell grid_cell(x, y);
	  const index_t grid_ind = cellIndex(filtered->info, grid_cell);
	  int gain_sum = 0;
	  int value_sum = 0;
	  // Run kernel
	  for (int kx = kernelmin; kx <= kernelmax; kx++)
	    {
	      for (int ky = kernelmin; ky <= kernelmax; ky++)
		{
		  const Cell cell(x+kx, y+ky);
		  if (
		      (cell.x >= 0) &&
		      (cell.x < filtered->info.width) &&
		      (cell.y >= 0) &&
		      (cell.y < filtered->info.height)
		      )
		    {
		      const index_t ind = cellIndex(grid->info, cell);
		      if (grid->data[ind] != -1)
			{
			  const int distance = std::abs(kx) + std::abs(ky);
			  const int celldvalue = (grid->data[ind]-50);
			  const int cellgain = 1+kernelsize-distance;
			  if (cellgain > 0)
			    {
			      gain_sum += cellgain;
			      value_sum += cellgain*celldvalue;
			    }
			}
		    }
		}
	    }

	  if (gain_sum != 0)
	    filtered->data[grid_ind] = 50 + (value_sum / gain_sum);

	  //filtered->data[grid_ind] = gain_sum;
	}
    }
  return filtered;
}

} // namespace occupancy_grid_utils
