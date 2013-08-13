#!/usr/bin/env python
#*****************************************************************************
# import_gpgga_data
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
This script contains a class to import gpgga data
"""
# imports
import csv

# general defines
deg_to_rad = pi/180.0

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

class gpgga_import():
    def __init__(self):
        self.tm = tranmerc()
        self.tm.set_params (wgs84_a, wgs84_f, utm_origin_latitude, \
            utm32_central_meridian, utm_false_easting, utm32_false_northing, utm_scale_factor)

    def get_tranmerc(filename):
        (gps,cs_err) = nmea_file_import(filename)
        tranmerc = []
        for i in xrange(len(gps)):
            pos = self.tm.geodetic_to_tranmerc (gps[i][1]*deg_to_rad, gps[i][2]*deg_to_rad)
            tranmerc.append(pos)
        return (tranmerc)

    # verify NMEA message checksum
    def nmea_checksum_ok(msg):
        msg = msg.strip('$')
        msg = msg.strip('\n')
        data,cs_recv = msg.split('*', 1)
        cs = reduce(operator.xor, (ord(s) for s in data), 0)
        return (cs == int(cs_recv,16))

    # convert position to decimal degree
    def nmea_latlon(hem, dmm):
        dm,mfrac = dmm.split('.')
        d = int(dm)//100
        m = int(dm)%100
        m += float('0.%s' % mfrac)
        deg = d + m/60.0
        if hem == 'S' or hem == 'W':
            deg = -deg
        return deg

    # import $GPGGA messages from NMEA log
    def nmea_file_import(filename):
        gga = []
        cs_err = 0
        f = open(filename, 'r')
        nall = 0
        nsampled = 0
        for msg in f:
            if nsampled >= max_measurements:
                break
            if msg.find ('$') != -1 and msg.find('\n') != -1:
                data = msg.split(',')
                if (data[0] == '$GPGGA'):
                    nall += 1
                    if nall > skip_measurements:
                        if (nmea_checksum_ok(msg)):
                            nsampled += 1
                            fix = int(data[6])
                            if fix > 0:
                                tim = data[1]
                                lat = nmea_latlon (data[3],data[2])
                                lon = nmea_latlon (data[5],data[4])
                                sat = int(data[7])
                                hdop = float(data[8])
                                alt = float(data[9])
                                geo = float(data[11])
                                age = float(data[13])
                                gga.append([fix,lat,lon,sat,hdop,alt,geo,tim,age])
                            else:
                                gga.append([fix,'','','','','','',''])
                        else:
                            cs_err += 1
        return (gga,cs_err)
