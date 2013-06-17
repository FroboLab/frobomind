#!/bin/sh

# determine the script directory
APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR

scp robot@frobobox:frobowork/*.bag ..
