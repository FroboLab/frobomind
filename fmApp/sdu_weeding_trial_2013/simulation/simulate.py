#!/usr/bin/env python
#*****************************************************************************
# Waypoint navigation simulation
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

Revision
2013-06-20 KJ First version
"""

# imports
import signal
from sim_import import pose_data, wptnav_data, cmd_vel_data
from wptnav_plot import ab_map, vel_plot
from math import fabs, pi

# parameters
offset_e = -651180.0   
offset_n = -6133758.0 
reject_pose_around_origo = True
max_buffer_seconds = 120
history = 15 # [s]
plot_ab = True
plot_ab_size = 5
plot_yaw = True
plot_set_speed = True
plot_relative_coordinates = True
pose_file = 'sim_pose.txt'
pose_skip_lines =  11400
pose_max_lines = 0 # 0 = read to the end
wptnav_file = 'sim_wptnav_status.txt'
wptnav_skip_lines = 0
wptnav_max_lines = 0
wptnav_status_interval = 2
cmd_vel_left_file = 'sim_cmd_vel_left.txt'
cmd_vel_right_file = 'sim_cmd_vel_right.txt'
cmd_vel_skip_lines = 0
cmd_vel_max_lines = 0
vel_left_file = 'sim_velocity_left.txt'
vel_right_file = 'sim_velocity_right.txt'
vel_skip_lines = 0
vel_max_lines = 0
sim_step_interval = 0.01 # 100 Hz
steps_btw_plot_updates = 100
steps_btw_yaw_plot_points = 10
min_gnss_fix_msg_before_pose_plot = 5 # used to avoid the high initial variance causes odd plots
save_images = False
window_size = 8.0

# wptnav_status struct
TIME = 0
MODE = 1
B_E = 2
B_N = 3
A_E = 4
A_N = 5
POSE_E = 6
POSE_N = 7
DIST_B = 8
BEARING_B = 9
HEADING_ERR = 10
DIST_AB = 11
TARGET_E = 12
TARGET_N = 13
TARGET_DIST = 14
TARGET_BEARING = 15
TARGET_HEADING_ERR = 16
LINV = 17
ANGV = 18

# main
print 'Simulation of waypoint navigation'
print 'Press CTRL-C to cancel'

# define and install ctrl-c handler
ctrl_c = 0
def signal_handler(signal, frame):
    global ctrl_c
    ctrl_c = 1
    print 'Ctrl-C pressed'
    print 'Quit'
signal.signal(signal.SIGINT, signal_handler)

# return angle within [-pi;pi[
def angle_limit (angle):
	while angle < -pi:
		angle += 2*pi
	while angle >= pi:
		angle -= 2*pi
	return angle

# setup plotting 
if plot_ab:
	ab_plot = ab_map(history, plot_ab_size, offset_e, offset_n, max_buffer_seconds, save_images)

if plot_set_speed:
	vel_plot = vel_plot (history, window_size, max_buffer_seconds, save_images)

# import simulation data
pose_sim = pose_data(pose_file, pose_skip_lines, pose_max_lines)
wptnav_sim = wptnav_data(wptnav_file, wptnav_skip_lines, wptnav_max_lines)
cmd_vel_sim = cmd_vel_data(cmd_vel_left_file, cmd_vel_right_file, cmd_vel_skip_lines, cmd_vel_max_lines)
vel_sim = cmd_vel_data(vel_left_file, vel_right_file, vel_skip_lines, vel_max_lines)
nav_mode = -2
nav_b_e = 0.0
nav_b_n = 0.0
nav_a_e = 0.0
nav_a_n = 0.0
wptnav_count = 0
mstr = 'INV'
latest_cmd_vel_l = 0.0
latest_cmd_vel_r = 0.0
latest_vel_l = 0.0
latest_vel_r = 0.0


# define simulation time based on odometry data
sim_offset = pose_sim.data[0][0]
sim_len = pose_sim.data[-1][0] - sim_offset
sim_steps = sim_len/sim_step_interval
sim_time = 0
print ('Simulation')
print ('  Step interval: %.2fs' % sim_step_interval)
print ('  Total: %.2fs (%.0f steps)' % (sim_len, sim_steps))

# run simulation
for step in xrange ((int(sim_steps)+1)):

	# simulation time housekeeping
	log_time = sim_time + sim_offset
	sim_time += sim_step_interval

    # update pose data
	(pose_updates, pose) = pose_sim.get_latest(log_time) 
	if pose_updates > 0:
		if reject_pose_around_origo == False or fabs(pose[1]) > 10.0 or fabs(pose[2]) > 10.0:
			latest_pose_yaw = pose[3]
			if plot_ab:
				ab_plot.append_pose(pose[0], pose[1], pose[2])

	# update wptnav status data
	(wptnav_updates, wn) = wptnav_sim.get_latest(log_time) 
	if wptnav_updates > 0:
		if plot_ab:
			ab_plot.append_wptnav_status(wn)

		# check for waypoint change
		if wn[B_E] != nav_b_e or wn[B_N] != nav_b_n or wn[A_E] != nav_a_e or wn[A_N] != nav_a_n:
			nav_b_e = wn[B_E]
			nav_b_n = wn[B_N]
			nav_a_e = wn[A_E]
			nav_a_n = wn[A_N]
			print  '%.3f Navigating to %.3f %.3f from %.3f %.3f' % (log_time, nav_b_e, nav_b_n, nav_a_e, nav_a_n)

		# check for mode change
		if wn[MODE] != nav_mode:
			nav_mode = wn[MODE]
			if nav_mode == 0:
				mstr = 'REM'
				print '%.3f Remote control' % (log_time)
			elif nav_mode == 1:
				mstr = 'DRV'
				print '%.3f Autonomous drive' % (log_time)
			if nav_mode == 2:
				mstr = 'TRN'
				print '%.3f Autonomus correction turn' % (log_time)
		wptnav_count += 1
		if wptnav_count % wptnav_status_interval == 0:
			print "%.3f %s b.dist %.2f b.head.err %5.1f ab.line.dist %.2f t.dist %4.2f t.head.err %4.1f cv %.2f cv.ang %.3f cv.l %.2f cv.r %.2f v.l %.2f v.r %.2f" \
				% (log_time, mstr, wn[DIST_B], wn[HEADING_ERR], wn[DIST_AB], \
				wn[TARGET_DIST], wn[TARGET_HEADING_ERR], wn[LINV], wn[ANGV], latest_cmd_vel_l, latest_cmd_vel_r, latest_vel_l, latest_vel_r)

	# update set_speed data
	(updates, l) = cmd_vel_sim.get_latest_left(log_time) 
	if updates > 0:
		vel_plot.append_cmd_vel_l(log_time, l[1])
		latest_cmd_vel_l = l[1]
	(updates, l) = cmd_vel_sim.get_latest_right(log_time) 
	if updates > 0:
		vel_plot.append_cmd_vel_r(log_time, l[1])
		latest_cmd_vel_r = l[1]
	(updates, l) = vel_sim.get_latest_left(log_time) 
	if updates > 0:
		vel_plot.append_vel_l(log_time, l[1])
		latest_vel_l = l[1]
	(updates, l) = vel_sim.get_latest_right(log_time) 
	if updates > 0:
		vel_plot.append_vel_r(log_time, l[1])
		latest_vel_r = l[1]


	# output to screen
	#print ('Step %d time %.2f log time %.2f' % (step+1, sim_time, (sim_time+sim_offset)))
	"""if step % steps_btw_yaw_plot_points == 0:
		if plot_pose_yaw:
			plot.append_pose_yaw (latest_pose_yaw)
		if plot_gnss_yaw:
			plot.append_gnss_yaw (latest_absolute_yaw)
		if plot_ahrs_yaw:
			plot.append_ahrs_yaw (latest_ahrs_yaw)
		if plot_odo_yaw:
			plot.append_odo_yaw (latest_odo_yaw)
	"""
	if step % steps_btw_plot_updates == 0:
		ab_plot.update()
		vel_plot.update()

    # exit if CTRL-C pressed
	if ctrl_c:
		break

# quit the simulation
if ctrl_c == False:
	print 'Simulation completed, press Enter to quit'
	ab_plot.save_last_plot()
	raw_input() # wait for enter keypress 

