#!/bin/sh

POLYGON_MAP=FlakkebjergOuterParcelCorners.csv

# determine the script directory
APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR

# copy waypoints and maps
cp ../maps/$POLYGON_MAP ~/.ros/polygon_map.txt

roslaunch ../launch/remote_view_polygon_map.launch

