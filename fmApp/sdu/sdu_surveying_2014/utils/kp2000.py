#!/usr/bin/env python
#*****************************************************************************
# KP2000 projection conversion
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
This class implements conversion between geodetic coordinates and the
KP2000 projection.

The class utilizes the tranmerc class located in transverse_mercator.py
The functions do not check for out of range or errors in input.

geodetic_to_kp2000 (latitude, longitude, projection)
    latitude: Accepted range is [-90;90] [deg]
    longitude: Accepted range is [-180;180] [deg]
    projection: kp2000j/kp2000s/kp2000b
    Returns: easting [m], northing [m]

kp2000_to_geodetic (easting, northing, projection)
    projection: kp2000j/kp2000s/kp2000b
    Returns: geodetic latitude [deg], geodetic longitude [deg]

Revision
2013-04-05 KJ Library created
"""

# imports
from math import pi
from transverse_mercator import tranmerc

# WGS-84 defines
wgs84_a = 6378137.0 # WGS84 semi-major axis of ellipsoid [m]
wgs84_f = 1/298.257223563 # WGS84 flattening of ellipsoid

# KP2000 defines
origin_latitude = 0.0
central_meridian = [9.5, 12.0, 15.0]
false_easting = [200000.0, 500000.0, 900000.0]
false_northing = 0.0
scale_factor = [0.99995, 0.99995, 1.0]

#*****************************************************************************
class kp2000conv():
    def __init__(self):
	self.kp2000j = 0 # KP2000 Jylland (Jutland) & Fyn (Funen) projection 
	self.kp2000s = 1 # KP2000 Sjaelland (Sealand) projection
	self.kp2000b = 2 # KP2000 Bornholm projection
        self.deg_to_rad = pi/180.0
        self.rad_to_deg = 180.0/pi
	self.central_meridian = []
	for i in xrange(len(central_meridian)):
	    self.central_meridian.append(central_meridian[i]*self.deg_to_rad)
	self.tm = tranmerc()

    def geodetic_to_kp2000 (self, latitude, longitude, projection):
	# set parameters for KP2000
	self.tm.set_params (wgs84_a, wgs84_f, origin_latitude, \
            self.central_meridian[projection], false_easting[projection], \
            false_northing, scale_factor[projection])

	# perform conversion and return KP2000 projection easting and northing
	return self.tm.geodetic_to_tranmerc \
            (latitude*self.deg_to_rad, longitude*self.deg_to_rad)
 
    def kp2000_to_geodetic (self, easting, northing, projection):
	# set parameters for KP2000
	self.tm.set_params (wgs84_a, wgs84_f, origin_latitude, \
            self.central_meridian[projection], false_easting[projection], \
            false_northing, scale_factor[projection])

	# perform conversion
        (lat,lon) = self.tm.tranmerc_to_geodetic (easting, northing)

	# return geodetic latitude and longitude in degrees
	return (lat*self.rad_to_deg, lon*self.rad_to_deg)

#*****************************************************************************
