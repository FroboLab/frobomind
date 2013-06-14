#!/usr/bin/env python
#/****************************************************************************
# Waypoint tranform
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
#****************************************************************************/
"""
This scipt was created to transform the first stint turn sequence to a nearby
grass field. The coordinates are not rotated.

2013-06-13 KJ First version
"""

import csv

in_file = 'stint_one_first_turn.txt'
out_file = 'stint_one_first_turn_grass_field.txt'

offset_e = 651185.181 - 651087.4919
offset_n = 6133764.492 - 6133775.7449 

# read in the waypoint list
file = open(in_file, 'r')
wpts = []
lines = csv.reader(file, delimiter='\t')
for easting, northing, wptid in lines:
	wpts.append([float(easting), float(northing), wptid])
file.close()

# transform waypoints
for i in xrange(len(wpts)):
	wpts[i][0] += offset_e
	wpts[i][1] += offset_n

# write the waypoint list
file = open(out_file, 'w')
for i in xrange(len(wpts)):
	file.write ('%.3f\t%.3f\t%s\n' % (wpts[i][0], wpts[i][1], wpts[i][2]))
file.close()


