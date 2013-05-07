#!/usr/bin/env python
#*****************************************************************************
# Pose 2D Estimator - robot drive simulation
# Copyright (c) 2013, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name FroboMind nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#*****************************************************************************
"""
This file contains a Python script to test the Pose 2D Estimator using system
feedback and sensor data from test drives with the Armadillo Pichi field 
robot platform: http://www.frobomind.org/index.php?title=Robot:Armadillo_Pichi

Revision
2013-04-23 KJ First version
"""

# imports
import signal
from math import sqrt, pi
from pose_2d_estimator import pose_2d_gnss_preprocessor, pose_2d_ekf
from sim_import import odometry_data, gnss_data
from pose_2d_estimator_plot import estimator_plot

# parameters
odo_file = 'sim_odometry.txt'
odo_max_lines = 9000 # read the entire file
gnss_file = 'sim_gnss.txt'
gnss_max_lines = 0 # read the entire file
sim_step_interval = 0.01 # 100 Hz
relative_coordinates = True # first gnss coordinate set to (0,0)
steps_btw_plot_updates = 500 # 5 seconds

# return signed difference between new and old angle
def angle_diff (angle_new, angle_old):
	diff = angle_new - angle_old
	while diff < -pi:
		diff += 2*pi
	while diff > pi:
		diff -= 2*pi
	return diff

# main
print 'Simulation of robot motion'
print 'Press CTRL-C to cancel'

# define and install ctrl-c handler
ctrl_c = 0
def signal_handler(signal, frame):
    global ctrl_c
    ctrl_c = 1
    print 'Ctrl-C pressed'
    print 'Quit'
signal.signal(signal.SIGINT, signal_handler)

# load plot functions
plot = estimator_plot()
plot.enable_odometry (True)
plot.enable_gnss (True)
plot.enable_pose (True)

# import simulation data
odo_sim = odometry_data(odo_file, odo_max_lines)
odometry = []
gnss_sim = gnss_data(gnss_file, gnss_max_lines, relative_coordinates)
gnss = []

# define simulation time based on odometry data
sim_offset = odo_sim.data[0][0]
sim_len = odo_sim.data[-1][0] - sim_offset
sim_steps = sim_len/sim_step_interval
sim_time = 0
print ('Simulation')
print ('  Step interval: %.2fs' % sim_step_interval)
print ('  Total: %.2fs (%.0f steps)' % (sim_len, sim_steps))

# initialize estimator (gnss preprocessing)
robot_max_speed = 3.0 # [m/s]

wgs84_a = 6378137.0 # Equatorial radius, generic for WGS-84 datum
wgs84_f = 1/298.257223563 # Flattening, generic for WGS-84 datum
utm_fe = 500000.0 # False Easting, generic for UTM projection
utm_scale = 0.9996 # Scale Factor, generic forUTM projection
utm_orglat = 0.0 # Origin Latitude, generic for UTM projection
utm32_cmer = 9.0 * pi/180.0 # By convention UTM32 (Central Meridian = 9deg) covers all of Denmark 
utm32_fn =  0.0 # Northern hemisphere
gp = pose_2d_gnss_preprocessor (robot_max_speed, wgs84_a, wgs84_f, utm_orglat, utm32_cmer, utm_fe, utm32_fn, utm_scale)
gp.set_auto_offset (True)

# initialize estimator (EKF)
prev_odometry = [0.0, 0.0, 0.0, 0.0] # [time,X,Y,theta]
ekf = pose_2d_ekf()
ekf.set_initial_guess([1.0, 1.0, 180.0*pi/180.0])

# run simulation
for step in xrange ((int(sim_steps)+1)):

	# simulation time housekeeping
	log_time = sim_time + sim_offset
	sim_time += sim_step_interval

    # update odometry position
	(odo_updates, odometry) = odo_sim.get_latest(log_time) 
	if odo_updates > 0:		
		odometry[1] *=4.5 # BUT WHY??????????
		odometry[2] *=4.5
		# EKF system update (odometry)
		delta_dist =  sqrt((odometry[1]-prev_odometry[1])**2 + (odometry[2]-prev_odometry[2])**2)
		delta_angle = angle_diff (odometry[3], prev_odometry[3])
		var_dist = 0.0000000001
		var_angle = 0.001
		prev_odometry = odometry
		pose = ekf.system_update (delta_dist, var_dist, delta_angle, var_angle)

		# plot update
		plot.append_odometry_position(odometry[1], odometry[2])
		plot.append_pose_position (pose[0]+1.0, pose[1]+1.0)
		# print "  odometry: %.3f %.3f %.3f, %3f, %3f" % (odometry[0], odometry[1], odometry[2], delta_dist, delta_angle)

    # update GNSS position
	(gnss_updates, gnss_measurement) = gnss_sim.get_latest(log_time)
	if gnss_updates > 0: # if new data available
		if gnss_measurement[3] > 0: # if satellite fix
			# GNSS data preprocessing
			pos = gp.add_gnss_measurement (gnss_measurement)
			if pos != False: # if no error
				var_pos = gp.estimate_variance()

				# EKF measurement update (GNSS)
				pose = ekf.measurement_update_gnss (pos, var_pos)

				# plot update
				plot.append_gnss_position(pos[0], pos[1])
				# print "  gnss: %.3f, %.3f %.3f" % (gnss_measurement[0], pos[0], pos[1])

	# output to screen
	#if step % 2000 == 0: # each 20 seconds, needs to be calculated!!!
	#	print ('Step %d time %.2f log time %.2f' % (step+1, sim_time, (sim_time+sim_offset)))
	if step % steps_btw_plot_updates == 0:
		plot.update()

    # exit if CTRL-C pressed
	if ctrl_c:
		break

# quit the simulation
if ctrl_c == False:
	print 'Simulation completed, press Enter to quit'
	raw_input() # wait for enter keypress 
	plot.save('map_pose_and_gps.png')

