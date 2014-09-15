This readme file contains various notes on the sdu_rsd_2014 application.

First version: 2014-08-14 by Kjeld Jensen <kjeld@frobomind.org>
Updated for RSD: 2014-09-15 by Kjeld Jensen <kjeld@frobomind.org>



WAYPOINT NAVIGATION EXAMPLE
Run bin/rsd_sim_wpt_nav to view a simulation of autonomous waypoint
navigation based on odometry pose estimation.

Run bin/rsd_run_wpt_nav to run the autonomous waypoint navigation on
a FrobitPro robot.

Run bin/rsd_sim_wpt_nav_feedback to view a simulation of autonomous
waypoint navigation using a FroboMind implemented model of the Frobit
(frobit_sim component). The model provides detailed controller feedback
which is presented in the simulation.

The waypoint navigation is fully implemented. When running the scripts the
file waypoints.txt in the sdu_rsd_2014/waypoints directory is copied
to the ROS working directory and the waypoint list is navigated when
switching to autonomous mode (see the KEYBOARD CONTROL section)


KEYBOARD CONTROL
The mission file keyboard_mission_node.py provides simple mission handling
based on keyboard input from the user:

a(uto)   Enter autonomous mode
m(anual) Enter manual mode

e(nable) Enable deadman signal (required for the robot to move)
Space:   Disable deadman signal

In manual mode
s(top)   Stop the robot by setting the velocity to zero



STAGE CONFIGURATION WHEN USING ROS GROOVY
The ROS package name of the stage simulator has changed from ROS Groovy to
ROS Hydro. The sdu_rsd_2014 launch files are configured for ROS Hydro
so if you are using ROS Groovy you need to make a minor modification to 
the rsd_sim_* launch files:

Simply replace pkg="stage_ros" by "pkg="stage" in the launch files.

