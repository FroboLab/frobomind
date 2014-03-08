#!/bin/sh

WAYPOINT_LIST=ArmadilloRouteNorthernPartStintOne.csv
#WAYPOINT_LIST=stint_one_first_turn_grass_field.txt

POLYGON_MAP=FlakkebjergOuterParcelCorners.csv

# determine the script directory
APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR

# copy waypoints and maps
cp ../waypoint_lists/$WAYPOINT_LIST ~/.ros/waypoints.txt
cp ../maps/$POLYGON_MAP ~/.ros/polygon_map.txt

# set up multi computer parameters
# export ROS_MASTER_URI=http//localhost:11311
export ROS_HOSTNAME=frobobox

# run FroboMind
source ~/frobowork/devel/setup.bash
roslaunch ../launch/navigate_waypoints.launch

