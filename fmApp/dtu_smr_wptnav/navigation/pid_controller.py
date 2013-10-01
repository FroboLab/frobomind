#!/usr/bin/env python
#/****************************************************************************
# PID Controller
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
2013-06-08 KJ First version
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
		self.integral_max = 0.0

	def set_parameters (self, Kp, Ki, Kd, integral_max):
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd
		self.integral_max = integral_max

	def reset(self):
		self.error_prev = 0.0
		self.integral = 0.0

	def update(self, error):
		# integration
		self.integral += error * self.dT # integrate error over time
		if self.integral > self.integral_max: # keep integral element within max
			self.integral = self.integral_max

		# derivation
		self.derivative = (error - self.error_prev)/self.dT # error change

		self.output = self.Kp*error + self.Ki*self.integral + self.Kd*self.derivative
		self.error_prev  = error # save err for next iteration
		return self.output

