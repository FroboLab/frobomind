#!/bin/sh

# change dir to the location of the shell script
APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR

rm *.png
rm sim_*.txt
rm plot_targets/*.jpg
rm plot_track/*.jpg

