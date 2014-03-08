#!/usr/bin/env python
#*****************************************************************************
# Larsen Landmaaler Service waypoint list format import utility
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
Revision
2013-12-19 KJ First version
"""

from sys import argv

# parameters
in_dec_seperator = ','
out_dec_seperator = '.'
seperator = ';'

def load_from_csv (filename):
	print 'Loading waypoints from: %s' % filename
	wpts = []
	lines = [line.rstrip('\n') for line in open(filename)] # read the file and strip \n
	wpt_num = 0
	for i in xrange(len(lines)): # for all lines
		if len(lines[i]) > 0 and lines[i][0] != '#': # if not a comment or empty line
			line = lines[i].replace (',','.')
			data = line.split (seperator) # split into comma separated list
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

argc = len(argv)
if argc != 3:
	print 'Usage: import_lls_format.py infile outfile'
else:
	inf =  argv[1:][0]
	outf =  argv[1:][1]

	out_wpts = []

	wpts = load_from_csv(inf)
	save_tranmerc_to_csv(outf, wpts, '# Easting,Northing')
	print 'Quit'

