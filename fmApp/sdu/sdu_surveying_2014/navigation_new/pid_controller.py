#!/usr/bin/env python
#/****************************************************************************
# PID Controller
# Copyright (c) 2013-2014, Kjeld Jensen <kjeld@frobomind.org>
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
2013-06-08 KJ First version
2013-12-12 KJ Changed integral_max to max_output
2014-02-21 KJ Added latest_update_values()
"""

class pid_controller():
	def __init__(self, dT):
		self.error = 0.0
		self.error_prev = 0.0
		self.integral = 0.0
		self.deriative = 0.0
		self.dT = dT
		self.Kp = 0.0
		self.Ki = 0.0
		self.Kd = 0.0
		self.feed_forward = 0.0
		self.p = 0.0
		self.i = 0.0
		self.d = 0.0
		self.max_output = 0.0
		self.first_time = True

	def set_parameters (self, Kp, Ki, Kd, feed_forward, max_output):
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd
		self.feed_forward = feed_forward
		self.max_output = max_output

	def reset(self):
		self.error_prev = 0.0
		self.integral = 0.0
		self.first_time = True

	def update(self, error):
		# proportional
		self.error = error
		self.p = self.Kp*self.error

		# integration
		self.integral += self.error * self.dT # integrate error over time
		self.i = self.Ki*self.integral

		# derivation
		if self.first_time == False:
			self.derivative = (self.error - self.error_prev)/self.dT # error change
			self.d = self.Kd*self.derivative
		else:
			self.d = 0.0
			self.first_time = False
		self.error_prev  = self.error # save err for next iteration

		# calculate feed_forward
		if self.error < 0:
			self.ff = -self.feed_forward
		else:
			self.ff = self.feed_forward

		# calculate output
		self.output = self.ff + self.p + self.i + self.d

		if self.output > self.max_output: # keep output element within +/- max
			self.output = self.max_output
		elif self.output < -self.max_output:
			self.output = -self.max_output

		return self.output

	def latest_update_values (self):
		return ([self.error, self.output, self.p, self.i, self.d, self.ff])

