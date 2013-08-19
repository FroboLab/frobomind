#!/usr/bin/env python
#*****************************************************************************
# plot_data
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
"""

import csv
from numpy import array
import matplotlib.pyplot as plt
from pylab import plot, title, xlabel, ylabel, show, subplot, ylim, grid, ion


max_controller_command = 100



ion()

# import data from csv file into a list
status = []
i = 0
file = open('debug_prop_mod_status.txt', 'r')
data = csv.reader(file, delimiter=',')
for time, voltage, current, power in data:
    status.append([]) 
    status[i].append (float(time))
    status[i].append (float(voltage))
    status[i].append (float(current))
    status[i].append (float(power))
    i = i + 1
file.close()

i = 0
deadman = []
file = open('debug_deadman.txt', 'r')
data = csv.reader(file, delimiter=',')
for time, deadman_button in data:
    deadman.append([]) 
    deadman[i].append (float(time))
    deadman[i].append (int(deadman_button))
    i = i + 1
file.close()


# convert to the array type
a_status = array(status)

plt.figure (1)
# plot the array
plot(a_status.T[0], a_status.T[1],'black')
plot(a_status.T[0], a_status.T[2],'r')
#plot(a.T[0], a.T[3],'g')
title ('Title')
xlabel('X-axis')
ylabel('Y-axis')


fb_left = []
i = 0
file = open('debug_feedback_left_status.txt', 'r')
data = csv.reader(file, delimiter=',')
for time, vel, cmd_vel, thrust in data:
    fb_left.append([]) 
    fb_left[i].append (float(time))
    fb_left[i].append (float(vel))
    fb_left[i].append (float(cmd_vel))
    fb_left[i].append (float(thrust)/max_controller_command)
    i = i + 1
file.close()
fb_right = []
i = 0
file = open('debug_feedback_right_status.txt', 'r')
data = csv.reader(file, delimiter=',')
for time, vel, cmd_vel, thrust in data:
    fb_right.append([]) 
    fb_right[i].append (float(time))
    fb_right[i].append (float(vel))
    fb_right[i].append (float(cmd_vel))
    fb_right[i].append (float(thrust)/max_controller_command)
    i = i + 1
file.close()

plt.figure (2)
a_fb_left = array(fb_left)
a_fb_right = array(fb_right)
a_deadman = array(deadman)

subplot (311) # three rows, one colum, plot 1
p_left_cmd_vel = plot(a_fb_left.T[0], a_fb_left.T[2],'black')
p_left_vel = plot(a_fb_left.T[0], a_fb_left.T[1],'blue')
p_left_thrust_ = plot(a_fb_left.T[0], a_fb_left.T[3],'red')
ylim([-1.1,1.1])
ylabel('left')
grid (True)
subplot (312) # three rows, one colum, plot 2
p_right_cmd_vel = plot(a_fb_right.T[0], a_fb_right.T[2],'black')
p_right_vel = plot(a_fb_right.T[0], a_fb_right.T[1],'blue')
p_right_thrust = plot(a_fb_right.T[0], a_fb_right.T[3],'red')
ylim([-1.1,1.1])
ylabel('right')
grid (True)
subplot (313) # three rows, one colum, plot 3
p_deadman = plot(a_deadman.T[0], a_deadman.T[1],'red')
ylim([-1,2])
ylabel('deadman')
grid (True)
xlabel('Time')
  

show()
raw_input("Press Enter to continue...")
