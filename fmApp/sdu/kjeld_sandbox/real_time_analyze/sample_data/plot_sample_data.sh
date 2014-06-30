#!/bin/sh

# change dir to the location of the shell script
APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR

# run the plot script
../scripts/plot_data.py

