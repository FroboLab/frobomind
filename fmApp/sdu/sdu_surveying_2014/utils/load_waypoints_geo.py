#!/usr/bin/env python

from pylab import *
from sys import argv

file_in = argv[1:][0]
file_out = argv[1:][1]
header = '#Survey waypoint list: Easting, Northing (UTM32)'


from transverse_mercator import tranmerc
from kp2000 import kp2000conv

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

tm = tranmerc()
tm.set_params (wgs84_a, wgs84_f, utm_origin_latitude, utm32_central_meridian, utm_false_easting, utm32_false_northing, utm_scale_factor)
kp = kp2000conv()


def load_waypoints_geo(filename):
	pts = []
	lines = [line.rstrip('\n') for line in open(filename)] # read the file and strip \n
	for i in xrange(len(lines)): # for all lines
		if len(lines[i]) > 0 and lines[i][0] != '#': # if not a comment or empty line
			if lines[i][:7] == '\tPoint ':
				data = lines[i][7:].split (',') # split into comma separated list
				name = data[0]
				kp2000_n = float (data[1])
				kp2000_e = float (data[2])
				alt = float (data[3])
				(lat, lon) = kp.kp2000_to_geodetic (kp2000_e, kp2000_n, kp.kp2000j)
				(e, n) = tm.geodetic_to_tranmerc (lat*deg_to_rad, lon*deg_to_rad)
				pts.append([name, e, n, kp2000_e, kp2000_n, alt])
			#else:
			#	print 'Erroneous point'
	return pts

# load track from the total station
wpts = load_waypoints_geo(file_in)
wpts_len = len(wpts)
print 'Waypoints: %d' % wpts_len

f = open(file_out, 'w')
if header != '':
	f.write ('%s\n' % header)
for i in xrange(len(wpts)):
	f.write ("%.4f,%.4f,,%s\n" % (wpts[i][1],wpts[i][2],wpts[i][0])) 
f.close()	

