#!/bin/sh

WAYPOINT_LIST=waypoints_sdu_grass_square.txt

# determine the script directory
APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR

# copy waypoints
cp ../waypoint_lists/$WAYPOINT_LIST ~/.ros/waypoints.txt

# set up multi computer parameters
# export ROS_MASTER_URI=http//localhost:11311
export ROS_HOSTNAME=localhost

# run FroboMind
#source ~/frobowork/devel/setup.bash
roslaunch ../launch/navigate_waypoints.launch

