#!/bin/sh

WAYPOINT_LIST=waypoints.txt

APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR

# copy the waypoint list
echo Using $WAYPOINT_LIST
cp ../waypoint_lists/$WAYPOINT_LIST ~/.ros/waypoints.txt

export ROS_MASTER_URI=http://frobobox:11311
export ROS_HOSTNAME=kjmac

roslaunch ../launch/remote_view_track_map.launch

