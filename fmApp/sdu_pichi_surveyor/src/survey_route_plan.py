#!/usr/bin/env python
#*****************************************************************************
# Route plan class
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
This class implements functions to import a route plan from a removable media
to a local directory and load a route plan from a local directory to a list.

Revision
2013-04-08 KJ Library created
"""

# imports
import os
from shutil import copyfile
import subprocess

# defines
removable_media_root = '/media'

err_ok = 0
err_removable_media = 1
err_copy = 2
err_not_found	= 3

class route_plan():
	def __init__(self):
		self.removable_media_root = removable_media_root
		self.err = err_ok
		self.err_txt = ['Ok', 'Removable media with rute plan not found', 'Unable to copy file', 'File not found']

	def import_route_plan_from_removable_media (self, file_name, source_dir, route_plan_dir, try_unmount):
		self.err = err_removable_media
 
        # test for existence of the specified file at each removable media
		removable_medias = os.listdir (self.removable_media_root)
		
		# loop through all mounted medias
		for test_dir in removable_medias: 
			if self.err == err_removable_media:

				# test for file existence by trying to open the file
				test_path = removable_media_root+'/'+test_dir+'/'+source_dir+'/'+file_name
				try:
					with open(test_path, "r") as f:
						f.close()
						found_path = test_path
						media_dir = test_dir
						self.err = err_ok
				except IOError:
					pass # do nothing		

		# if file found at removable media
		if self.err == err_ok:

			# copy file to destination path (relative to script path)
			dest_path = os.path.dirname(os.path.abspath(__file__))+'/'+route_plan_dir+'/'+file_name
			try:				
				copyfile(found_path, dest_path)
			except IOError:
				self.err = err_copy

			if try_unmount == True and self.err == err_ok:
				# try unmounting the removable media if requested
				if try_unmount == True:
					umount_cmd = 'umount '+removable_media_root+'/'+media_dir
					try:
						subprocess.call(umount_cmd, shell=True)
					except IOError:
						pass # do nothing

	def load_route_plan (self, file_name, route_plan_dir, separation_char):
		self.err = err_not_found
		# try opening the file and read the content
		path = os.path.dirname(os.path.abspath(__file__))+'/'+route_plan_dir+'/'+file_name
		try:
			with open(path, "r") as f:
				file_content = f.readlines()
				f.close()
				self.err = err_ok
		except IOError:
			file_content = ''

		rte = []
		# if route file found
		if self.err == err_ok:
			for i in xrange (len(file_content)):
				line = file_content[i].strip()
				if line != '' and line[0] != '#': # if line not empty or a comment
					line = line.strip(separation_char) # strip any trailing separation chars
					rte.append (line.split(separation_char))
		return rte

	def error (self):
		return self.err

	def error_text (self):
		return self.err_txt[self.err]

