#!/bin/sh

APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR

rm ../*.bag
rm ../data*.txt
rm ../*.png

