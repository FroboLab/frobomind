#!/usr/bin/env python
#*****************************************************************************
# KP2000 projection conversion test
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
This file contains a simple Python script to test the KP2000 projection
conversion class.

Revision
2013-04-05 KJ First version
"""
# import kp2000conv class
from transverse_mercator.kp2000 import kp2000conv
from math import pi, cos

# define test position
test_lat =  55.0000000000
test_lon = 009.0000000000
test_lat =  55.832245358
test_lon = 009.507804944
print 'Test position [deg]:'
print '  latitude:  %.10f'  % (test_lat)
print '  longitude: %.10f'  % (test_lon)

# instantiate kp2000conv class
kp = kp2000conv()

# convert from geodetic to KP2000
(easting, northing) = kp.geodetic_to_kp2000 (test_lat, test_lon, kp.kp2000j)
print '\nConverted from geodetic to KP2000 [m]'
print '  %.5fe %.5fn' % (easting, northing)

# convert back from KP2000 to geodetic
(lat, lon) = kp.kp2000_to_geodetic (easting, northing, kp.kp2000j)
print '\nConverted back to geodetic [deg]:'
print '  latitude:  %.10f'  % (lat)
print '  longitude: %.10f'  % (lon)

# detrmine conversion position error [m]
lat_err = abs(lat-test_lat)
lon_err = abs(lon-test_lon)
earth_radius = 6378137.0 # [m]
lat_pos_err = lat_err/360.0 * 2*pi*earth_radius
lon_pos_err = lon_err/360.0 * 2*pi*(cos(lat)*earth_radius)
print '\nPositional error from the two conversions [m]:'
print '  latitude:  %.9f'  % (lat_pos_err)
print '  longitude: %.9f'  % (lon_pos_err)


