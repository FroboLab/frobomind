#!/bin/bash

source /home/henrik/roswork/devel/setup.bash
cd /home/henrik/roswork/src/fmApp/sdu_lidar3d_demo/scripts/

# Start roscore
xterm -hold -e "./startDemo01.sh" &

sleep 5

# Start sdu_lidar3d_demo
xterm -hold -e "./startDemo02live.sh" &
