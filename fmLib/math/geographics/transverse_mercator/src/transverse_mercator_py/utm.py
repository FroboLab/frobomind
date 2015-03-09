#!/usr/bin/env python
#*****************************************************************************
# Universal Transverse Mercator (UTM) conversion
# Copyright (c) 2013-2015, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
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
Universal Transverse Mercator (UTM) projection.

The class utilizes the tranmerc class located in transverse_mercator.py
The functions do not check for out of range or errors in input.

set_zone_override (zone)
    use to override the default zone by one of its neighbouring zones. If
    more distant zone is chosen, the inaccuracies will be significant.

geodetic_to_utm (latitude, longitude)
    latitude: Accepted range is [-90;90] [deg]
    longitude: Accepted range is [-180;180] [deg]
    Returns: hemisphere, zone, letter, easting [m], northing [m]

utm_to_geodetic (hemisphere, zone, easting, northing)
    hemisphere: 'N' or 'S' accepted
    zone: Valid UTM zone accepted
    Returns: geodetic latitude [deg], geodetic longitude [deg]

Revision
2013-04-05 KJ Library created
2015-03-09 KJ Minor update of the license text.
"""

# imports
from math import pi
from transverse_mercator import tranmerc

# WGS-84 defines
wgs84_a = 6378137.0 # WGS84 semi-major axis of ellipsoid [m]
wgs84_f = 1/298.257223563 # WGS84 flattening of ellipsoid

# UTM defines
utm_false_easting = 500000.0
utm_scale_factor = 0.9996
utm_origin_latitude = 0.0

#*****************************************************************************
class utmconv():
    def __init__(self):
	self.false_e = 500000.0
	self.false_n = 0.0
	self.scale = 0.9996
	self.zone_override = 0
        self.deg_to_rad = pi/180.0
        self.rad_to_deg = 180.0/pi
	self.tm = tranmerc()

    def set_zone_override (self, zone):
	# allow manual override of the utm zone
	self.zone_override = zone

    def geodetic_to_utm (self, latitude, longitude):
	lat = latitude*self.deg_to_rad
	lon = longitude*self.deg_to_rad
        lat_deg_int = int(latitude)
        lon_deg_int = int(longitude)

	# if manually override to a neighbouring zone 
        if self.zone_override > 0:
            zone = self.zone_override
	else:
   	    # calculate the zone based on the longitude
            zone = int((longitude + 180)/6) + 1
	    # handle areas with special conventions (Denmark & South West Norway)
            if (lat_deg_int>55) and (lat_deg_int<64) and (lon_deg_int> -1) and (lon_deg_int< 3):
                zone = 31
            if (lat_deg_int>55) and (lat_deg_int< 64) and (lon_deg_int> 2) and (lon_deg_int< 12):
                zone = 32
   	    # handle areas with special conventions (Svalbard)
            if lat_deg_int > 71:
	        if (lon_deg_int>-1) and (lon_deg_int<9):
                    zone = 31
                if (lon_deg_int>8) and (lon_deg_int<21):
                    zone = 33
                if (lon_deg_int>20) and (lon_deg_int<33):
                    zone = 35
                if (lon_deg_int>32) and (lon_deg_int<42):
                    zone = 37

	# calculate central meridian for this zone 
	central_meridian = ((zone - 1)*6 - 180 + 3)*self.deg_to_rad        

	# set false northing based on hemishpere
        if latitude >= 0.0: 
            false_northing = 0
            hemisphere = 'N'
	    # determine the UTM zone letter
     	    if  latitude >= 72.0: zlet = 'X'
            elif latitude >= 64.0: zlet = 'W'
	    elif latitude >= 56.0: zlet = 'V'
	    elif latitude >= 48.0: zlet = 'U'
	    elif latitude >= 40.0: zlet = 'T'
	    elif latitude >= 32.0: zlet = 'S'
	    elif latitude >= 24.0: let = 'R'
	    elif latitude >= 16.0: zlet = 'Q'
	    elif latitude >= 8.0: zlet = 'P'
	    else: zlet = 'N'
        else:
            false_northing = 10000000
            hemisphere = 'S'

	    # determine the UTM zone letter
	    if latitude >= -8.0: zlet = 'M'
	    elif latitude >= -16.0: zlet = 'L'
	    elif latitude >= -24.0: zlet = 'K'
	    elif latitude >= -32.0: zlet = 'J'
	    elif latitude >= -40.0: zlet = 'H'
	    elif latitude >= -48.0: zlet = 'G'
	    elif latitude >= -56.0: zlet = 'F'
	    elif latitude >= -64.0: zlet = 'E'
	    elif latitude >= -72.0: zlet = 'D'
	    else: zlet = 'C'

	# set parameters for WGS-84, UTM, the false northing and the zone central meridian
	self.tm.set_params (wgs84_a, wgs84_f, utm_origin_latitude, central_meridian, utm_false_easting, false_northing, utm_scale_factor)

	# perform conversion
	(easting, northing) = self.tm.geodetic_to_tranmerc (lat, lon)

	# return hemisphere, utm zone, utm letter, easting and northing
 	return (hemisphere, zone, zlet, easting, northing)
 
    def utm_to_geodetic (self, hemisphere, zone, easting, northing):

	# calculate the central meridian for the zone
	central_meridian = ((zone - 1)*6 - 180 + 3)*self.deg_to_rad      

	# determine the false northing based on the hemisphere
        if hemisphere == 'N':
            false_northing = 0
        else:
            false_northing = 10000000

	# set parameters for WGS-84, UTM, the false northing and the zone central meridian
	self.tm.set_params (wgs84_a, wgs84_f, utm_origin_latitude, central_meridian, utm_false_easting, false_northing, utm_scale_factor)

	# perform conversion
        (lat,lon) = self.tm.tranmerc_to_geodetic (easting, northing)

	# return geodetic latitude and longitude in degrees
	return (lat*self.rad_to_deg, lon*self.rad_to_deg)

#*****************************************************************************
