#!/bin/sh

APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR
cd ..
echo Loading rosbag: $1
$APPDIR/export_pose_estimation.py $1
$APPDIR/plot_gnss_track.py data_gnss.txt
#$APPDIR/cleanup.sh

