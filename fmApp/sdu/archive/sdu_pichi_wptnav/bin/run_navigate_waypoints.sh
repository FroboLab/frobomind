#!/bin/sh

APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR

roslaunch ../launch/navigate_waypoints.launch

