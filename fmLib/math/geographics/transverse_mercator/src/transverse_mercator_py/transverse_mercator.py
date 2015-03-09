#!/usr/bin/env python
#*****************************************************************************
# Transverse Mercator projection
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
The tranmerc implements conversion between geodetic coordinates and the
Transverse Mercator projection.
 
A substantial portion of the algorithms used in this class are derived from
an implemtation in C by the U.S. Army Topographic Engineering Center,
Geospatial Information Division. The C source files state that no license
apply to those. The implementation is based on the references:

  "Handbook for Transformation of Datums, Projections, Grids and Common
   Coordinate Systems. TEC-SR-7. U.S. Army Topographic Engineering Center. 1996."

  "Map Projections - A Working Manual, U.S. Geological Survey Professional
   Paper 1395, by John P. Snyder, 1987"

It should be noted that newer implementations of algorithms for the
Transverse Mercator projection that are more accurate and more efficient are
avilable. For instance the GeographicLib TransverseMercator (C++) documented
here: 

  "Charles F. F. Karney. Transverse Mercator with an accuracy of a few
   nanometers. J. Geodesy 85(8), 475-485 (Aug. 2011)
   http://arxiv.org/abs/1002.1417 "


set_params( a, f, origin_latitude, central_meridian, false_easting,
false_northing, scale_factor)
    Specifying a particular ellipsoid:
        a: Semi-major axis, radius [m] at the Equator
        f: Ratio of the difference between the semi-major axis and polar
           radius of the Earth to its semi-major axis.

    Specifying the particular variation of the Transverse Mercator projection:
        central_meridian: Longitude [-pi;pi] [rad] at the origin of the
                          projection.
        origin_latitude:  Latitude [rad] at the origin of the projection.
        false_easting:    Coordinate value [m] assigned to the central
                          meridian of the projection to avoid the
                          inconvenience of using negative coordinates.
        false_northing:   Coordinate value [m] assigned to the origin latitude
                          of the projection to avoid the inconvenience of using
                          negative coordinates.
        scale_factor:     Multiplier for reducing a distance in projected
                          coordinates to the actual distance along the central
                          meridian.

geodetic_to_tranmerc (lat, lon)
    lat: Latitude, accepted ange is [-pi/2;pi/2] radians
    lon: Longitude, accepted range is [-pi;pi] radians

The functions do not check for out of range or errors in input.

More information about the Transverse Mercator projection may be found at
http://en.wikipedia.org/wiki/Transverse_Mercator_projection

Revision
2013-04-05 KJ Library created
2015-03-09 KJ Minor update of the license text.
"""
# imports
from math import pi, sqrt, sin, cos, tan, fabs

#*****************************************************************************
class tranmerc():
    def __init__(self):
        self.deg_to_rad = pi/180.0
        self.rad_to_deg = 180.0/pi

    def set_params (self, a, f, origin_latitude, central_meridian, false_easting, false_northing, scale_factor):
        self.a = a 
        self.f = f 
        self.central_meridian = central_meridian
        self.origin_lat = origin_latitude 
        self.false_e = false_easting 
        self.false_n = false_northing
        self.scale = scale_factor 

        self.origin_lon = central_meridian
        self.es = 2*self.f - self.f**2 # eccentricity squared
        self.ebs = self.es/(1 - self.es) # second eccentricity squared
        self.b = self.a * (1 - self.f)
        tn = (self.a - self.b)/(self.a + self.b)
        tn2 = tn**2
        tn3 = tn**3
        tn4 = tn**4
        tn5 = tn**5
        self.ap = self.a * (1.0 - tn + 5.0 * (tn2 - tn3)/4.0 + 81.0 * (tn4 - tn5)/64.0)
        self.bp = 3.0 * self.a * (tn - tn2 + 7.0 * (tn3 - tn4)/8.0 + 55.0 * tn5/64.0 )/2.0
        self.cp = 15.0 * self.a * (tn2 - tn3 + 3.0 * (tn4 - tn5 )/4.0) /16.0
        self.dp = 35.0 * self.a * (tn3 - tn4 + 11.0 * tn5 / 16.0) / 48.0
        self.ep = 315.0 * self.a * (tn4 - tn5) / 512.0

    def geodetic_to_tranmerc (self, lat, lon):
        dlam = lon - self.origin_lon;

        if dlam > pi:
            dlam -= 2*pi
        if dlam < -pi:
            dlam += 2*pi
        if fabs(dlam) < 2.e-10:
            dlam = 0.0
        s = sin(lat)
        c = cos(lat)
        c2 = c**2
        c3 = c**3
        c5 = c**5
        c7 = c**7
        t = tan (lat)
        tan2 = t**2
        tan3 = t**3
        tan4 = t**4
        tan5 = t**5
        tan6 = t**6
        eta = self.ebs * c2
        eta2 = eta**2
        eta3 = eta**3
        eta4 = eta**4

        sn = self.sphsn(lat) 
        tmd = self.sphtmd(lat) 
        tmdo = self.sphtmd(self.origin_lat) # origin

        # northing
        t1 = (tmd - tmdo) * self.scale
        t2 = sn * s * c * self.scale/2.0
        t3 = sn * s * c3 * self.scale * (5.0 - tan2 + 9.0 * eta + 4.0 * eta2) /24.0
        t4 = sn * s * c5 * self.scale * (61.0 - 58.0 * tan2 + tan4 + 270.0 * eta - 330.0 * tan2 * eta + 445.0 * eta2 + 324.0 * eta3 -680.0 * tan2 * eta2 + 88.0 * eta4 -600.0 * tan2 * eta3 - 192.0 * tan2 * eta4) / 720.0;
        t5 = sn * s * c7 * self.scale * (1385.0 - 3111.0 * tan2 + 543.0 * tan4 - tan6) / 40320.0
        northing = self.false_n + t1 + pow(dlam,2.0) * t2 + pow(dlam,4.0) * t3 + pow(dlam,6.0) * t4 + pow(dlam,8.0) * t5

        # easting
        t6 = sn * c * self.scale
        t7 = sn * c3 * self.scale * (1.0 - tan2 + eta ) /6.0
        t8 = sn * c5 * self.scale * (5.0 - 18.0 * tan2 + tan4 + 14.0 * eta - 58.0 * tan2 * eta + 13.0 * eta2 + 4.0 * eta3 - 64.0 * tan2 * eta2 - 24.0 * tan2 * eta3 )/ 120.0
        t9 = sn * c7 * self.scale * ( 61.0 - 479.0 * tan2 + 179.0 * tan4 - tan6 ) /5040.0
        easting = self.false_e + dlam * t6 + pow(dlam,3.0) * t7 + pow(dlam,5.0) * t8 + pow(dlam,7.0) * t9;
        return (easting, northing)

    def tranmerc_to_geodetic (self, easting, northing):
        tmdo = self.sphtmd(self.origin_lat) # true Meridional Distances for latitude of origin
        tmd = tmdo + (northing - self.false_n)/self.scale # origin 
        sr = self.sphsr(0.0) # first estimate
        ftphi = tmd/sr
	for i in xrange (5):
            t10 = self.sphtmd (ftphi)
            sr = self.sphsr(ftphi)
            ftphi += (tmd - t10)/sr
        sr = self.sphsr(ftphi) # radius of Curvature in the meridian
        sn = self.sphsn(ftphi) # radius of Curvature in the meridian

        s = sin(ftphi) # sine cosine terms
        c = cos(ftphi)
        t = tan(ftphi) # tangent value
        tan2 = t**2
        tan4 = t**4
        eta = self.ebs * c**2
        eta2 = eta**2
        eta3 = eta**3
        eta4 = eta**4
        de = easting - self.false_e
        if fabs(de) < 0.0001:
            de = 0.0
            
        # calculate the latitude
        t10 = t / (2.0 * sr * sn * self.scale**2)
        t11 = t * (5.0  + 3.0 * tan2 + eta - 4.0 * eta**2 - 9.0 * tan2 * eta) / (24.0 * sr * sn**3 * self.scale**4)
        t12 = t * (61.0 + 90.0 * tan2 + 46.0 * eta + 45.0 * tan4 - 252.0 * tan2 * eta  - 3.0 * eta2 + 100.0 * eta3 - 66.0 * tan2 * eta2 - 90.0 * tan4 * eta + 88.0 * eta4 + 225.0 * tan4 * eta2 + 84.0 * tan2* eta3 - 192.0 * tan2 * eta4) / (720.0 * sr * sn**5 * self.scale**6)
        t13 = t * ( 1385.0 + 3633.0 * tan2 + 4095.0 * tan4 + 1575.0 * t**6)/ (40320.0 * sr * sn**7 * self.scale**8)
        lat = ftphi - de**2 * t10 + de**4 * t11 - de**6 * t12 + de**8 * t13;

	# calculate the longitude
        t14 = 1.0 / (sn * c * self.scale)
        t15 = (1.0 + 2.0 * tan2 + eta) / (6.0 * sn**3 * c * self.scale**3)
        t16 = (5.0 + 6.0 * eta + 28.0 * tan2 - 3.0 * eta2 + 8.0 * tan2 * eta + 24.0 * tan4 - 4.0 * eta3 + 4.0 * tan2 * eta2 + 24.0 * tan2 * eta3) / (120.0 * sn**5 * c * self.scale**5)
        t17 = (61.0 +  662.0 * tan2 + 1320.0 * tan4 + 720.0 * t**6) / (5040.0 * sn**7 * c * self.scale**7)
        dlam = de * t14 - de**3 * t15 + de**5 * t16 - de**7 * t17 # difference in longitude
        lon = self.origin_lon + dlam
            
        while lat > pi/2.0:
            lat = pi - lat
            lon += pi
            if lon > pi:
                lon -= 2*pi

        while lat < -pi/2.0:
            lat = -(lat+pi)
            lon += pi
            if lon > pi:
                lon -= 2*pi
            
        if lon > 2*pi:
            lon -= 2*pi
        if lon < -pi:
            lon += 2*pi

        return (lat, lon)

    def sphsn (self, lat): 
	return self.a/sqrt(1.0 - self.es*sin(lat)**2)

    def sphtmd (self, lat):
	return self.ap*lat - self.bp*sin(2.0*lat) + self.cp*sin(4.0*lat) - self.dp*sin(6.0*lat) + self.ep*sin(8.0*lat)

    def sphsr (self, lat):
        return  self.a*(1.0 - self.es)/self.denom(lat)**3

    def denom (self, lat):
        return sqrt(1.0 - self.es*sin(lat)**2)

#*****************************************************************************
