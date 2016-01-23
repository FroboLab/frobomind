This Frobomind component contains an obstacle detect algorithm based on input
from a lidar.

The purpose of the component is provide the frobit robot with sensor based
knowledge about obstacles in front of the robot. The component implements
a warning and an alarm threshold barrier in front of the robot each at a width
of the obstacle_scan_width launch parameter times two.

The output topic /fmKnowledge/obstacle contains two integers, one for the left
and one for the right half of the barriers. A value of 0 means no breach, 
1 means warning and 2 means alert.

The algorithm has been kept simple to lower the computation time. It may
easily be replaced by a more robust algorithm and the component should thus
be considered an example or template for further development.

First version: 2016-01-18 by Kjeld Jensen <kjeld@frobomind.org>


