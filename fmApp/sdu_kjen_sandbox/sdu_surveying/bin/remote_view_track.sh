#!/bin/sh

WPTLIST='waypoints_sdu_s.txt'
#WPTLIST='square_2m.txt'

APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR

# copy the waypoint list
echo Using $WAYPOINT_LIST
cp ../waypoints/$WPTLIST ~/.ros/waypoints.txt

export ROS_MASTER_URI=http://frobobox:11311
export ROS_HOSTNAME=kjmac

roslaunch ../launch/remote_view_track_map.launch

