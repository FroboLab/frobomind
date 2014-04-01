#!/bin/sh


# determine the script directory
APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR

# delete previous time-lapse images in "~/.ros
rm ~/.ros/img*.jpg

# copy waypoints and maps

# set up multi computer parameters
export ROS_MASTER_URI="http://localhost:11311"
# export ROS_HOSTNAME=kjmac

# run FroboMind
roslaunch ../launch/sim_navigate_waypoints.launch

