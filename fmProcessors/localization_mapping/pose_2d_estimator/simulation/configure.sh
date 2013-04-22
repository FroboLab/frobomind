#!/bin/sh

APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR

# export PYTHONPATH=$PYTHONPATH:~/frobowork/src/fmLib/math/geographics/transverse_mercator/src/transverse_mercator/
# ./robot_drive_sim.py

ln -s ~/frobowork/src/fmLib/math/geographics/transverse_mercator/src/transverse_mercator/transverse_mercator.py 
ln -s ~/frobowork/src/fmLib/math/geographics/transverse_mercator/src/transverse_mercator/utm.py 
