#!/usr/bin/env python
#*****************************************************************************
# Polygon Map - Simulation
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
"""py
Revision
2013-05-31 KJ First version
"""

from polygon_map import polygon_map
from polygon_map_plot import polygon_map_plot
import csv
import copy

# initialize the polygon map
polymap = polygon_map()
polymap.set_nearby_threshold (10.0) 
polymap.set_polygons_per_update (1600)

# initialize the polygon map plot
map_title = "Field"
map_window_size = 10.0 # [inches]
easting_offset = 0.0 # [m]
northing_offset = 0.0 # [m]
easting_offset = -588815.0 # [m]
northing_offset = -6137299.0 # [m]
polyplot = polygon_map_plot(map_title, map_window_size, easting_offset, northing_offset)

# import polygons 
file = open('TekInnerParcelCornersExtended.csv', 'r')
file_content = csv.reader(file, delimiter='\t')
for name,e1,n1,e2,n2,e3,n3,e4,n4 in file_content:
	polygon = [[float(e1),float(n1)],[float(e2),float(n2)],[float(e3),float(n3)],[float(e4),float(n4)]]
	polymap.add_polygon (name, polygon)
	polyplot.add_polygon (copy.deepcopy(polygon))
file.close()

print "Polygons imported: %ld" % polymap.poly_total

polyplot.update_map_plot ()


#for i in range (10):
#	print 'polygon', polymap.poly_pts[i]

# other stuff
polymap.update_pos (588833.5, 6137216.5)
print polymap.nearby
print 'nummer 5', polymap.update_map(10000.0)
print polymap.nearby

polymap.update_pos (588842.1,6137220.2)
print 'nummer 39', polymap.update_map(20000.0)
print polymap.nearby


print 'Simulation completed, press Enter to quit'
#raw_input() # wait for enter keypress 
#polyplot.draw_polygon_within (5)
#polyplot.update_map_plot ()


#raw_input() # wait for enter keypress 
#polyplot.save_map_plot()
