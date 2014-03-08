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
		print 'Importing float data'
		file = open(filename, 'r')
		file_content = csv.reader(file, delimiter=',')
	 	self.data = []
		i = 0
		for time, x in file_content:
			if i > skip_lines:
				if data_type == 'float':
					self.data.append([float(time), float(x)])
				elif data_type == 'int':
					self.data.append([float(time), int(x)])
			i += 1
			if max_lines > 0 and i == max_lines:
				break
		file.close()
		self.length = len(self.data)
		print '\tTotal samples: %d' % (self.length) 

	def trim_begin (self, begin):
		while self.data[0][0] < begin:	
			del (self.data[0])
		for i in xrange(len(self.data)):
			self.data[i][0] -= begin

	def trim_end (self, end):
		while self.data[-1][0] > end:	
			del (self.data[-1])

	def get_latest(self, time):
		new_data = 0
		while self.i < self.length and self.data[self.i][0] <= time:
			self.i += 1
			new_data += 1
		return (new_data, self.data[self.i-1])

# import data
skip_lines = 0
max_lines = 99999999
rt = scalar_data('rosbag_rt_timing.txt', 'int', skip_lines, max_lines)
cpu = scalar_data('rosbag_cpu_load.txt', 'float', skip_lines, max_lines)
mem = scalar_data('rosbag_memory_load.txt', 'float', skip_lines, max_lines)

# determine first and last time stamp
time_begin = rt.data[0][0]
if cpu.data[0][0] > time_begin:
	time_begin = cpu.data[0][0]
if mem.data[0][0] > time_begin:
	time_begin = mem.data[0][0]

time_end = rt.data[-1][0]
if cpu.data[-1][0] < time_end:
	time_end = cpu.data[-1][0]
if mem.data[-1][0] < time_end:
	time_end = mem.data[-1][0]

# trim data
rt.trim_begin(time_begin)
cpu.trim_begin(time_begin)
mem.trim_begin(time_begin)
rt.trim_end(time_end)
cpu.trim_end(time_end)
mem.trim_end(time_end)

print "Duration %.2f seconds" % (time_end - time_begin)

# convert CPU load to %
for i in xrange(len(cpu.data)):
	cpu.data[i][1] *= 100

tim = []
clock = -3
actual = 0

timing_adjust = 1.000094

for i in xrange(len(rt.data)):
	actual += (rt.data[i][1]/timing_adjust)
	clock += 100
	tim.append ([rt.data[i][0], 10+(actual-clock)/10.0])

rtperc = []
for i in xrange(len(rt.data)):
	rtperc.append ([rt.data[i][0], rt.data[i][1]/timing_adjust/10.0])


cpuT = zip(*cpu.data)
timT = zip(*tim)
rtT = zip(*rtperc)

avg = 0
mini = 9999
maxi = 0
for i in xrange(len(rtperc)):
	avg += rtperc[i][1]
	if rtperc[i][1] > maxi:
		maxi = rtperc[i][1]
	if rtperc[i][1] < mini:
		mini = rtperc[i][1]
avg /= len(rtperc)
print 'samples %ld' % len(rtperc)
print 'average %.6f' % avg 
print 'min/max %.2f %.2f' % (mini,maxi) 

vari = 0
for i in xrange(len(rtperc)):
	vari += (rtperc[i][1] - avg)**2
vari /= len(rtperc)
print 'variance %.2f' % vari 


hist,bins=np.histogram(array(rtT[1]),bins=50)
width=1*(bins[1]-bins[0])
center=(bins[:-1]+bins[1:])/2

ion()

plt.bar(center,hist,align='center',width=width)
savefig('hist.png')


# plot data

subplot (211)
title ('CPU load & scheduler intervals')
cpuT = zip(*cpu.data)
cpu_plt = plot(cpuT[0], cpuT[1], 'red')
#memT = zip(*mem.data)
#mem_plt = plot(memT[0], memT[1], 'black')

ylim([0.0, 100.0])
ylabel('CPU load [%]')
grid (True)

subplot (212)
#ylim([0, 20])
#timT = zip(*tim)
#tim_plt = plot(timT[0], timT[1], 'black')
rt_plt = plot(rtT[0], rtT[1], 'black')
ylabel('Scheduler interval [ms]')
xlabel('Time [s]')
grid (True)


'''subplot (211)
title ('CPU & memory load')
cpu_plt = plot(cpuT[0], cpuT[1], 'red')
#memT = zip(*mem.data)
#mem_plt = plot(memT[0], memT[1], 'black')

ylim([0.0, 100.0])
ylabel('CPU load [%]')
grid (True)
title ('CPU load & scheduler intervals')

subplot (212)
#ylim([-5, 5])
tim_plt = plot(timT[0], timT[1], 'black')
ylabel('100 Hz sched. [ms]')
grid (True)
'''
draw()

raw_input() # wait for enter keypress 
savefig('cpu_load_sched_interval.png')

