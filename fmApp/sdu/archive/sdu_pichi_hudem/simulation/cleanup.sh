#!/bin/sh

APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR

rm *.bag
rm sim_*.txt
rm *.pyc
rm *.png

