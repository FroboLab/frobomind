#!/bin/sh

APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR

export PYTHONPATH=$APPDIR/../src:$PYTHONPATH

./simulate.py

