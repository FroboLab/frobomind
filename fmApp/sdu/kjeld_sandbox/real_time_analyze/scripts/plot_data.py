#!/usr/bin/env python
#*****************************************************************************
# Plot real time analyze data
# Copyright (c) 2014, Kjeld Jensen <kjeld@frobomind.org>
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

import csv
import matplotlib.pyplot as plt
from pylab import plot, axis, grid, title, xlabel, ylabel, xlim, ylim, draw, show, subplot, ion, savefig
import numpy as np
from numpy import *


class scalar_data():
	def __init__(self, filename, data_type, skip_lines, max_lines):
		self.i = 0
		file = open(filename, 'r')
		file_content = csv.reader(file, delimiter=',')
	 	self.time = []
	 	self.data = []
		i = 0
		for time, x in file_content:
			if i > skip_lines:
				self.time.append(float(time))
				if data_type == 'float':
					self.data.append(float(x))
				elif data_type == 'int':
					self.data.append(int(x))
			i += 1
			if max_lines > 0 and i == max_lines:
				break
		file.close()
		self.length = len(self.data)

	def trim_begin (self, begin):
		while self.time[0] < begin:	
			del (self.time[0])
			del (self.data[0])
		for i in xrange(len(self.time)):
			self.time[i] -= begin

	def trim_end (self, end):
		while self.time[-1] > end: 
			del (self.time[-1])
			del (self.data[-1])

	def convert_to_percent(self):
		for i in xrange(len(self.data)):
			self.data[i] *= 100

	def convert_100ns_to_ms(self):
		for i in xrange(len(self.data)):
			self.data[i] /= 10.0

def trim_begin_end (rt, cpu, mem):
	# determine first and last time stamp
	time_begin = rt.time[0]
	if cpu.time[0] > time_begin:
		time_begin = cpu.time[0]
	if mem.time[0] > time_begin:
		time_begin = mem.time[0]

	time_end = rt.time[-1]
	if cpu.time[-1] < time_end:
		time_end = cpu.time[-1]
	if mem.time[-1] < time_end:
		time_end = mem.time[-1]

	# trim data
	rt.trim_begin(time_begin)
	cpu.trim_begin(time_begin)
	mem.trim_begin(time_begin)
	rt.trim_end(time_end)
	cpu.trim_end(time_end)
	mem.trim_end(time_end)

def adjust_rt_timing(rt):
	a = array(rt.data)
	mean = a.mean()
	offset = mean/10.0
	print 'rt timing adjusted by %.9f ms' % offset
	for i in xrange(len(rt.data)):
		rt.data[i] /= offset

def calc_rt_timing_delays (rt):
	delay = []
	clock_true = 0
	clock_actual = 0.0
	for i in xrange(len(rt.data)):
		clock_true += 10
		clock_actual += (rt.data[i]) 
		delay.append(clock_actual-clock_true) 

	# offset adjust by assuming that the lowest experienced delay is 0 ms
	minval = 10000000
	for i in xrange(len(delay)):
		if delay[i] < minval:
			minval = delay[i]

	for i in xrange(len(delay)):
		delay[i] -= minval
	return delay

def calc_statistics (delay):
	a = array(delay)
	np_mean = a.mean()
	np_var = a.var()

	print 'samples %ld' % len(delay)
	print 'mean %.9f' % np_mean
	print 'variance %.6f' % np_var
	print 'minimum/maximum %.2f %.2f' % (min(a), max(a)) 
	p95 = np.percentile (a, 95.)
	print ('95 %% percentile: %.19f' % (p95))

# import data
skip_lines = 0
max_lines = 99999999

# timing_adjust compensates for inaccuracy in measurement, due to x-tal on
# the ATmega it actually counts 1.000094 per 100 ns. 
#timing_adjust = 1.000094 # theoretical (measured using a scope)

pc_rt = scalar_data('rosbag_rt_timing.txt', 'int', skip_lines, max_lines)
pc_cpu = scalar_data('rosbag_cpu_load.txt', 'float', skip_lines, max_lines)
pc_mem = scalar_data('rosbag_memory_load.txt', 'float', skip_lines, max_lines)
trim_begin_end (pc_rt, pc_cpu, pc_mem) # trim begin and end times
pc_cpu.convert_to_percent() # convert cpu to percent
pc_mem.convert_to_percent() # convert memory to percent
pc_rt.convert_100ns_to_ms() # adjust til ms
adjust_rt_timing(pc_rt) # make sure that the mean is exactly 10 ms
pc_delay = calc_rt_timing_delays (pc_rt) # calc real time timing delays
calc_statistics (pc_delay) # print all statistics informations

ion()

plt.figure (1)
subplot (211)
title ('CPU load & scheduler intervals')
pc_cpu_plt = plot(pc_cpu.time, pc_cpu.data, 'red')
ylim([0.0, 100.0])
ylabel('CPU load [%]')
grid (True)

subplot (212)
rt_plt = plot(pc_rt.time, pc_delay, 'black')
ylabel('Scheduler interval [ms]')
xlabel('Time [s]')
grid (True)

savefig('fm_cpu_load_sched_intervals.png')

draw()

raw_input() # wait for enter keypress 

