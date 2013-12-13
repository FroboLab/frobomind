#!/usr/bin/env python
#*****************************************************************************
# KP2000 to UTM projection conversion
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
This file contains a Python script to convert coordinates in a file between
KP2000, UTM32 and geodetic coordinates.

Revision
2013-12-09 KJ First version
"""
# import kp2000conv class
from transverse_mercator import tranmerc
from kp2000 import kp2000conv
from math import pi, cos
from sys import argv

# parameters
seperator = ','

# general defines
deg_to_rad = pi/180.0
rad_to_deg = 180.0/pi
# WGS-84 defines
wgs84_a = 6378137.0 # WGS84 semi-major axis of ellipsoid [m]
wgs84_f = 1/298.257223563 # WGS84 flattening of ellipsoid
# UTM defines
utm_false_easting = 500000.0
utm_scale_factor = 0.9996
utm_origin_latitude = 0.0 * deg_to_rad
# UTM32 defines
utm32_central_meridian = 9.0 * deg_to_rad
utm32_false_northing = 0.0 * deg_to_rad

def load_from_csv (filename):
	print 'Loading waypoints from: %s' % filename
	wpts = []
	lines = [line.rstrip('\n') for line in open(filename)] # read the file and strip \n
	wpt_num = 0
	for i in xrange(len(lines)): # for all lines
		if len(lines[i]) > 0 and lines[i][0] != '#': # if not a comment or empty line
			data = lines[i].split (seperator) # split into comma separated list
			if len(data) >= 2 and data[0] != '' and data[1] != '':
				wpt_num += 1
				e = float (data[0])
				n = float (data[1])

				wpts.append([e, n])
			else:
				print '  Erroneous waypoint: %s' % lines[i]
	print '  Total %d waypoints loaded.' % wpt_num
	return wpts

def save_tranmerc_to_csv(filename, wpts, header):
	print 'Saving %d waypoints to: %s' % (len(wpts),filename)
	f = open(filename, 'w')
	if header != '':
		f.write ('%s\n' % header)
	for i in xrange(len(wpts)):
		f.write ("%.5f,%.5f\n" % (wpts[i][0],wpts[i][1])) 
	f.close()	

def save_ll_to_csv(filename, wpts, header):
	print 'Saving %d waypoints to: %s' % (len(wpts),filename)
	f = open(filename, 'w')
	if header != '':
		f.write ('%s\n' % header)
	for i in xrange(len(wpts)):
		f.write ("%013.10f,%014.10f\n" % (wpts[i][0],wpts[i][1])) 
	f.close()	

argc = len(argv)
if argc != 4:
	print 'Usage: convert_waypoints.py format infile outfile'
	print 'Format: ll_utm32/utm32_ll/ll_kp2000j/kp2000j_ll'
else:
	conv =  argv[1:][0]
	inf =  argv[1:][1]
	outf =  argv[1:][2]

	out_wpts = []

	if conv == 'll_utm32':
		print 'Convertion from geographical coordinates to UTM32...'
		in_wpts = load_from_csv(inf)
		tm = tranmerc()
		tm.set_params (wgs84_a, wgs84_f, utm_origin_latitude, utm32_central_meridian, utm_false_easting, utm32_false_northing, utm_scale_factor)
		for i in xrange(len(in_wpts)):
			(e, n) = tm.geodetic_to_tranmerc (in_wpts[i][0]*deg_to_rad, in_wpts[i][1]*deg_to_rad)
			out_wpts.append ([e,n])
		save_tranmerc_to_csv(outf, out_wpts, '# Easting,Northing (UTM32)')
		print 'Quit'

	elif conv == 'utm32_ll':
		print 'Convertion from UTM32 to geographical coordinates...'
		in_wpts = load_from_csv(inf)
		tm = tranmerc()
		tm.set_params (wgs84_a, wgs84_f, utm_origin_latitude, utm32_central_meridian, utm_false_easting, utm32_false_northing, utm_scale_factor)
		for i in xrange(len(in_wpts)):
			(lat, lon) = tm.tranmerc_to_geodetic (in_wpts[i][0], in_wpts[i][1])
			out_wpts.append ([lat*rad_to_deg, lon*rad_to_deg])
		save_ll_to_csv(outf, out_wpts, '# Latitude,Longitude')
		print 'Quit'

	elif conv == 'll_kp2000j':
		print 'Convertion from geographical coordinates to KP2000J...'
		in_wpts = load_from_csv(inf)
		kp = kp2000conv()
		for i in xrange(len(in_wpts)):
			(e, n) = kp.geodetic_to_kp2000 (in_wpts[i][0], in_wpts[i][1], kp.kp2000j)
			out_wpts.append ([e,n])
		save_tranmerc_to_csv(outf, out_wpts, '# Easting,Northing (KP2000J)')
		print 'Quit'

	elif conv == 'kp2000j_ll':
		print 'Convertion from KP2000J to geographical coordinates...'
		in_wpts = load_from_csv(inf)
		kp = kp2000conv()
		for i in xrange(len(in_wpts)):
			(lat, lon) = kp.kp2000_to_geodetic (in_wpts[i][0], in_wpts[i][1], kp.kp2000j)
			out_wpts.append ([lat, lon])
		save_ll_to_csv(outf, out_wpts, '# Latitude,Longitude')
		print 'Quit'

	else:
		print 'Invalid format: %s' % conv

