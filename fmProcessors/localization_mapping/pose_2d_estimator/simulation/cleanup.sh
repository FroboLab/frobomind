#!/bin/sh

APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR

rm *.bag
rm *.txt
rm *.pyc
rm *.png

