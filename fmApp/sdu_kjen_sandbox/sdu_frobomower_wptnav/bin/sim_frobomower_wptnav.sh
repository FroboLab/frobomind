#!/bin/sh

POLYGON_MAP=FlakkebjergOuterParcelCorners.csv

# determine the script directory
APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR

# copy waypoints and maps
cp ../maps/$POLYGON_MAP ~/.ros/polygon_map.txt

# set up multi computer parameters
export ROS_MASTER_URI="http://localhost:11311"
export ROS_HOSTNAME=kjmac

# run FroboMind
roslaunch ../launch/sim_navigate_waypoints.launch

