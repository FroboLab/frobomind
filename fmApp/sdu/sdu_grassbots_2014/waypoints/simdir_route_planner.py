#!/usr/bin/env python
#/****************************************************************************
# Simple and Dirty Route Planner!
# Copyright (c) 2014, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#	* Redistributions of source code must retain the above copyright
#	  notice, this list of conditions and the following disclaimer.
#	* Redistributions in binary form must reproduce the above copyright
#	  notice, this list of conditions and the following disclaimer in the
#	  documentation and/or other materials provided with the distribution.
#   * Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
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
#****************************************************************************/
"""
2014-09-17 KJ First version
"""

# imports
from pylab import *
from math import pi

#*****************************************************************************
# USER CONFIGURABLE PARAMETERS BEGIN HERE

# route plan parameters
number_of_rows = 8
row_length = 25.0 # [m]
row_lin_vel = 1.9 # [m/s]

headland_first_turn_is_left = True
headland_turn_radius = 2.0 # [m]
headland_add_length = 1.5 # [m]
headland_intermediate_points = 2 # per 90deg turn
headland_lin_vel = 0.4 # [m/s]

# translate to absolute ENU coordinates (set everything to zero for relative)
absolute_northing = 100.0 # [m]
absolute_easting = 50.0 # [m]
absolute_heading = pi/8.0 # [radians]

# USER CONFIGURABLE PARAMETERS END HERE
#*****************************************************************************

# options
simdir_version = '2014-09-17'
show_plot = True
save_file = True
file_name = 'waypoints.txt'

# csv file waypoint list structure
W_E = 0 
W_N = 1
W_HEADING = 2
W_ID = 3
W_NAV_MODE = 4
W_LIN_VEL = 5
W_ANG_VEL = 6
W_PAUSE = 7
W_TASK = 8
INVALID_DATA = -1000000

# return the vector v rotated by theta
def vec2d_rot (v, theta): 
	rot_x = v[0]*cos(theta) - v[1]*sin(theta)
	rot_y = v[0]*sin(theta) + v[1]*cos(theta)
	return ([rot_x, rot_y])

# headland turn template generator
def generate_headland_turn (turn_left, turn_radius, intermediate_points, straight_dist):
	turn = []	
	rad = pi/(2*(intermediate_points + 1)) # angle between intermediate points

	# first 90 degree turn to the left
	for i in xrange(intermediate_points+2):
		if i > 0:
			w = vec2d_rot([cos(i*rad)*turn_radius, sin(i*rad)*turn_radius], -pi/2.0)
			turn.append([w[0], w[1] + turn_radius])

	# go straight for a while
	turn.append ([turn[-1][0], turn[-1][1] + straight_dist])

	# second 90 degree turn to the left
	for i in xrange(intermediate_points+2):
		if i > 0:
			w = vec2d_rot([cos(i*rad)*turn_radius, sin(i*rad)*turn_radius], 0.0)
			turn.append([w[0], w[1] + turn_radius + straight_dist])

	# if right turn requested
	if turn_left == False:
		for i in xrange(len(turn)):
			turn[i][1] = -turn[i][1]
	return turn

# find coordinate minimum and maximum
def enu_min_max(w):
	min_e = 100000000000000
	min_n = 100000000000000
	max_e = -100000000000000
	max_n = -100000000000000
	for i in xrange(len(w)):
		if min_e > w[i][0]:
			min_e = w[i][0]
		elif max_e < w[i][0]:
			max_e = w[i][0]
		if min_n > w[i][1]:
			min_n = w[i][1]
		elif max_n < w[i][1]:
			max_n = w[i][1]
	return (min_e, max_e, min_n, max_n)

# generate left and right turn templates (format: 'r_e_l_t' means 'right_turn_left_end')
r_e_l_t = generate_headland_turn (True, headland_turn_radius, headland_intermediate_points, headland_add_length)
r_e_r_t = generate_headland_turn (False, headland_turn_radius, headland_intermediate_points, headland_add_length)
l_e_l_t = []
for i in xrange(len(r_e_l_t)):
	l_e_l_t.append(vec2d_rot([r_e_l_t[i][0],r_e_l_t[i][1]], pi))
l_e_r_t = []
for i in xrange(len(r_e_r_t)):
	l_e_r_t.append(vec2d_rot([r_e_r_t[i][0],r_e_r_t[i][1]], pi))

# prepare the route planning
wpt = []
current_row = 1
going_right = True
turn_left = headland_first_turn_is_left
e = 0.0
n = 0.0
rho = INVALID_DATA
def wpt_id():
	return  'wpt%d' % len(wpt)
ang_vel = INVALID_DATA
pause = 0.0
task = INVALID_DATA

# start coordinate
wpt.append ([e,n,rho,wpt_id(),'AB',row_lin_vel,ang_vel,pause,task])

while current_row <= number_of_rows:
	if going_right:
		# row navigation
		e += row_length
		wpt.append ([e,n,rho,wpt_id(),'AB',row_lin_vel,ang_vel,pause,task])
		# make a headland turn if there is at least one more row 
		if current_row + 1 < number_of_rows:
			if turn_left:
				for i in xrange(len(r_e_l_t)):
					wpt.append ([e+r_e_l_t[i][0],n+r_e_l_t[i][1],rho,wpt_id(),'PP',headland_lin_vel,ang_vel,pause,task])
				turn_left = False
			else:
				for i in xrange(len(r_e_r_t)):
					wpt.append ([e+r_e_r_t[i][0],n+r_e_r_t[i][1],rho,wpt_id(),'PP',headland_lin_vel,ang_vel,pause,task])
				turn_left = True
			e = wpt[-1][0]
			n = wpt[-1][1]
		going_right = False	
	else:
		# row navigation
		e -= row_length
		wpt.append ([e,n,rho,wpt_id(),'AB',row_lin_vel,ang_vel,pause,task])
		# make a headland turn if there is at least one more row 
		if current_row + 1 <= number_of_rows:
			if turn_left:
				for i in xrange(len(l_e_l_t)):
					wpt.append ([e+l_e_l_t[i][0],n+l_e_l_t[i][1],rho,wpt_id(),'PP',headland_lin_vel,ang_vel,pause,task])
				turn_left = False
			else:
				for i in xrange(len(l_e_r_t)):
					wpt.append ([e+l_e_r_t[i][0],n+l_e_r_t[i][1],rho,wpt_id(),'PP',headland_lin_vel,ang_vel,pause,task])
				turn_left = True
			e = wpt[-1][0]
			n = wpt[-1][1]
		going_right = True
	current_row	+= 1


# convert the generated route plan to absolute coordinates
for i in xrange(len(wpt)):
	w = vec2d_rot([wpt[i][0],wpt[i][1]], absolute_heading)
	wpt[i][0] = w[0]+absolute_easting
	wpt[i][1] = w[1]+absolute_northing

# save the list
if save_file == True:
	file = open(file_name, 'w')
	file.write ('# Simple and Dirty Route Planner %s\r\n' % simdir_version)
	file.write ('# easting,northing,heading,id,nav_mode,linear_vel,angular_vel,pause,task\r\n')
	for i in xrange(len(wpt)):
		e = wpt[i][W_E]
		n = wpt[i][W_N]	
		heading = wpt[i][W_HEADING]
		if heading != INVALID_DATA:
			heading_str = '%.3f' % heading
		else:
			heading_str = ''
 		name = wpt[i][W_ID]
		nav_mode_str = wpt[i][W_NAV_MODE]
		lin_vel = wpt[i][W_LIN_VEL]
		if lin_vel != INVALID_DATA:
			lin_vel_str = '%.3f' % lin_vel
		else:
			lin_vel_str = ''
		ang_vel = wpt[i][W_ANG_VEL]
		if ang_vel != INVALID_DATA:
			ang_vel_str = '%.3f' % ang_vel
		else:
			ang_vel_str = ''
		pause = wpt[i][W_PAUSE]
		if pause != INVALID_DATA:
			pause_str = '%.1f' % pause
		else:
			pause_str = ''

		task = wpt[i][W_TASK]
		if task != INVALID_DATA:
			task_str = '%d' % task
		else:
			task_str = ''
	
		file.write('%.3f,%.3f,%s,%s,%s,%s,%s,%s,%s\r\n' % (e, n, heading_str, name, nav_mode_str, lin_vel_str, ang_vel_str, pause_str, task_str))
	file.close()

# plot the route plan
if show_plot == True:
	print 'Plotting the waypoints'
	e_vals = [row[0] for row in wpt]
	n_vals = [row[1] for row in wpt]
	plot(e_vals, n_vals,'ro')
	plot(e_vals, n_vals,'r')
	(min_e, max_e, min_n, max_n) = enu_min_max(wpt)
	border = 1.0
	title ('Route Plan')
	xlabel('Easting [m]')
	ylabel('Northing [m]')
	axis('equal')
	grid (True)
	xlim(min_e-border, max_e+border)
	ylim(min_n-border, max_n+border)
	show()

