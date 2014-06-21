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
feedback and sensor data from FroboMind rosbags

Revision
2013-05-10 KJ First alpha release
2013-05-15 KJ Added AHRS support
2013-06-05 KJ Improved pre-processing algorithms 
"""

# imports
import signal
from math import sqrt, pi
from pose_estimator import odometry_gnss_pose_preprocessor, odometry_pose_ekf
from sim_import import odometry_data, imu_data, gnss_data
from robot_track_map import track_map

# parameters
ekf_easting_init = 588784.0   # set these EKF initial guess coordinates close
ekf_northing_init = 6137288.0 # to actual transverse mercator coordinates.
var_pos_init = 500.0**2
var_yaw_init = pi**2

plot_pose = True
plot_gnss = True
plot_odometry = True
plot_yaw = True
plot_pose_yaw = True # used only in this simulation to determine which values to append
plot_gnss_yaw = True # used only in this simulation to determine which values to append
plot_ahrs_yaw =  False # used only in this simulation to determine which values to append
plot_odo_yaw = True # used only in this simulation to determine which values to append
plot_relative_coordinates = True
odo_file = 'sim_odometry.txt'
odo_skip_lines = 0
odo_max_lines = 0 # 0 = read to the end
imu_file = 'sim_imu.txt'
imu_skip_lines = 0
imu_max_lines = 0
gnss_file = 'sim_gnss.txt'
gnss_skip_lines = 0
gnss_max_lines = 0 
sim_step_interval = 0.01 # 100 Hz
steps_btw_plot_updates = 100
steps_btw_yaw_plot_points = 100
min_gnss_fix_msg_before_pose_plot = 5 # used to avoid the high initial variance causes odd plots
var_dist = 0.001**2 # per system update
var_angle = 0.0059**2 # per system update
var_yaw =   0.0000000000001**2
pi2 = 2.0*pi
pose = [0.0, 0.0, 0.0]


# return angle within [0;2pi[
def angle_limit (angle):
	while angle < 0:
		angle += pi2
	while angle >= pi2:
		angle -= pi2
	return angle

# return signed difference between new and old angle
def angle_diff (angle_new, angle_old):
	diff = angle_new - angle_old
	while diff < -1.5*pi:
		diff += 2*pi
	while diff > 1.5*pi:
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

# setup plotting
if plot_relative_coordinates == False: # define absolute or relative plotting coordinates
	plot_easting_offset = 0.0
	plot_northing_offset = 0.0
else:
	plot_easting_offset = -ekf_easting_init 
	plot_northing_offset = -ekf_northing_init
plot = track_map(plot_pose, plot_gnss, plot_odometry, plot_yaw, \
	"Simulation", 8.0, plot_easting_offset, plot_northing_offset)
latest_pose_yaw = 0.0
latest_absolute_yaw = 0.0
latest_ahrs_yaw = 0.0
latest_odo_yaw = 0.0
gnss_fix_msg_count = 0 

# import simulation data
odo_sim = odometry_data(odo_file, odo_skip_lines, odo_max_lines)
imu_sim = imu_data(imu_file, imu_skip_lines, imu_max_lines)
gnss_sim = gnss_data(gnss_file, gnss_skip_lines, gnss_max_lines)

# define simulation time based on odometry data
sim_offset = odo_sim.data[0][0]
sim_len = odo_sim.data[-1][0] - sim_offset
sim_steps = sim_len/sim_step_interval
sim_time = 0
print ('Simulation')
print ('  Step interval: %.2fs' % sim_step_interval)
print ('  Total: %.2fs (%.0f steps)' % (sim_len, sim_steps))

# initialize estimator (preprocessing)
robot_max_speed = 3.0 # [m/s]
pp = odometry_gnss_pose_preprocessor (robot_max_speed)

# initialize estimator (EKF)
prev_odometry = [0.0, 0.0, 0.0, 0.0] # [time,X,Y,theta]
ekf = odometry_pose_ekf()
ekf.initial_guess ([ekf_easting_init, ekf_northing_init, 0], var_pos_init, var_yaw_init)

# run simulation
for step in xrange ((int(sim_steps)+1)):

	# simulation time housekeeping
	log_time = sim_time + sim_offset
	sim_time += sim_step_interval

    # update odometry position
	(odo_updates, odometry) = odo_sim.get_latest(log_time) 
	if odo_updates > 0:
		# odometry data preprocessing
		time_recv = odometry[0]
		delta_dist =  sqrt((odometry[1]-prev_odometry[1])**2 + (odometry[2]-prev_odometry[2])**2)
		delta_angle = angle_diff (odometry[3], prev_odometry[3])
		prev_odometry = odometry
		forward = odometry[4] >= 0
		pp.odometry_new_data (time_recv, delta_dist, delta_angle, forward)
		
		# EKF system update (odometry)
		pose = ekf.system_update (delta_dist, var_dist, delta_angle, var_angle)

		# odometry plot update
		latest_odo_yaw += delta_angle
		latest_odo_yaw = angle_limit(latest_odo_yaw)
		plot.append_odometry_position(odometry[1], odometry[2])

		# pose plot update
		if gnss_fix_msg_count >= min_gnss_fix_msg_before_pose_plot:
			plot.append_pose_position (pose[0], pose[1])
		latest_pose_yaw = angle_limit(pose[2])

    # update IMU data
	(imu_updates, imu_data) = imu_sim.get_latest(log_time) 
	if imu_updates > 0:		
		# EKF measurement update (IMU)
		time_recv = imu_data[0]
		rate_yaw = imu_data[1]
		orientation_yaw = angle_limit(imu_data[2])
		pos = pp.imu_new_data (time_recv, rate_yaw)
	
		# plot update
		latest_ahrs_yaw = orientation_yaw

    # update GNSS data
	(gnss_updates, gnss_data) = gnss_sim.get_latest(log_time)
	if gnss_updates > 0: # if new data available
		solution = gnss_data[5]
		if solution > 0: # if the new data has a satellite fix
			gnss_fix_msg_count += 1
			time_recv = gnss_data[0]
			easting = gnss_data[3] 
			northing = gnss_data[4]
			satellites = gnss_data[6]
			hdop = gnss_data[7]

			# GNSS error check
			#dist = sqrt((easting-pose[0])**2+(northing-pose[1])**2)
			#if dist > 5.0:
			#	print 'GPS-Post err: %.1f fix %d %.3f' % (dist, solution, time_recv)
	


			# GNSS data preprocessing
			valid = pp.gnss_new_data (time_recv, easting, northing, solution, satellites, hdop)
			if valid == True: # if position is validated
				var_pos = pp.gnss_estimate_variance_pos() # estimate the position variance

				yaw = pose[2]
				(yaw_valid, yaw_test) = pp.estimate_absolute_orientation()
				if yaw_valid == True:
					yaw = yaw_test
					latest_absolute_yaw = yaw

				pose = ekf.measurement_update ([easting, northing, yaw], var_pos, var_yaw) # EKF measurement update

				# plot update
				plot.append_gnss_position(easting, northing)


	# output to screen
	#if step % 2000 == 0: # each 20 seconds, needs to be calculated!!!
	#	print ('Step %d time %.2f log time %.2f' % (step+1, sim_time, (sim_time+sim_offset)))
	if step % steps_btw_yaw_plot_points == 0:
		if plot_pose_yaw:
			plot.append_pose_yaw (latest_pose_yaw)
		if plot_gnss_yaw:
			plot.append_gnss_yaw (latest_absolute_yaw)
		if plot_ahrs_yaw:
			plot.append_ahrs_yaw (latest_ahrs_yaw)
		if plot_odo_yaw:
			plot.append_odo_yaw (latest_odo_yaw)

	if step % steps_btw_plot_updates == 0:
		plot.update()

    # exit if CTRL-C pressed
	if ctrl_c:
		break

# quit the simulation
if ctrl_c == False:
	print 'Simulation completed, press Enter to quit'
	raw_input() # wait for enter keypress 
	plot.save('simulation')

