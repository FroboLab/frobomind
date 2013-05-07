#!/bin/sh

APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR

export PYTHONPATH=$APPDIR/../src:$APPDIR/../../../../fmLib/math/geographics/transverse_mercator/src/transverse_mercator:$PYTHONPATH

./simulate.py

