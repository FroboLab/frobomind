#!/usr/bin/env python
#*****************************************************************************
# Plot a list of waypoints
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
This file contains a Python script to plot a list of waypoints represented by
transverse mercator coordinates.

Revision
2013-12-09 KJ First version
"""
# imports
from sys import argv
#import matplotlib.pyplot as plt
from pylab import plot, subplot, axis, grid, title, xlabel, ylabel, xlim, ylim, draw, show, ion

# general defines
plot_canvas = 2.0

def load_from_csv (filename):
	print 'Loading waypoints from: %s' % filename
	wpts = []
	lines = [line.rstrip('\n') for line in open(filename)] # read the file and strip \n
	wpt_num = 0
	for i in xrange(len(lines)): # for all lines
		if len(lines[i]) > 0 and lines[i][0] != '#': # if not a comment or empty line
			data = lines[i].split (',') # split into comma separated list
			if len(data) >= 2 and data[0] != '' and data[1] != '':
				wpt_num += 1
				e = float (data[0])
				n = float (data[1])

				wpts.append([e, n])
			else:
				print '  Erroneous waypoint: %s' % lines[i]
	print '  Total %d waypoints loaded.' % wpt_num
	return wpts


argc = len(argv)
if argc != 2:
	print 'Usage: plot_waypoints.py infile'
else:
	inf =  argv[1:][0]

	w = load_from_csv(inf)
	
	if len(w) > 0:
		offset_e = w[0][0]
		offset_n = w[0][1]

	min_e = 10000000000000000
	min_n = 10000000000000000
	max_e = -10000000000000000
	max_n = -10000000000000000

	relw = []
	for i in xrange(len(w)):
		rel_e = w[i][0]-offset_e
		if rel_e < min_e:
			min_e = rel_e
		elif rel_e > max_e:
			max_e = rel_e
		rel_n = w[i][1]-offset_n
		if rel_n < min_n:
			min_n = rel_n
		elif rel_n > max_n:
			max_n = rel_n		
		relw.append([rel_e, rel_n])

	wT = zip(*relw)
	print max_n  + plot_canvas
	print 'Generating plot'	
	ion()
	wp, = plot(wT[0], wT[1], "ro")
	wp, = plot(wT[0], wT[1], "r")
	xlim([min_e - plot_canvas, max_e + plot_canvas])
	ylim([min_n - plot_canvas, max_n + plot_canvas])
	axis ('equal')					
	xlabel('Easting')
	ylabel('Northing')
	grid (True)
	title ('Waypoints')
	show()	
	print 'Offset: E%.0f, N%.0f' % (offset_e, offset_n) 
	print 'Press Enter to quit'
	raw_input() # wait for enter keypress 

