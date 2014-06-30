#!/bin/sh

WAYPOINT_LIST=waypoints.txt

# determine the script directory
APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR

# copy waypoints
cp ../waypoint_lists/$WAYPOINT_LIST ~/.ros/waypoints.txt

# set up multi computer parameters
# export ROS_MASTER_URI=http//localhost:11311
export ROS_HOSTNAME=localhost

# launch FroboMind
roslaunch ../launch/frobomower_wptnav.launch

