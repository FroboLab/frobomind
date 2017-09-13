#!/usr/bin/env python
# SLIP - Serial Line Internet Protocol class
# Made by Christian
# Inspired by https://github.com/reduzent/pyslip/blob/master/slip.py

class slip_protocol():
	def __init__(self):
		self.SLIP_END = "C0".decode("hex")
		self.SLIP_ESC = "DB".decode("hex")
		self.SLIP_ESC_END = "DC".decode("hex")
		self.SLIP_ESC_ESC = "DD".decode("hex")

	def encode(self, packet):
		# Encode an initial END character to flush out any data that
		# may have accumulated in the receiver due to line noise
		encoded = self.SLIP_END
		for char in packet:
			# SLIP_END
			if char == self.SLIP_END:
				encoded +=  self.SLIP_ESC + self.SLIP_ESC_END
			# SLIP_ESC
			elif char == self.SLIP_ESC:
				encoded += self.SLIP_ESC + self.SLIP_ESC_ESC
			# the rest can simply be appended
			else:
				encoded += char
		encoded += self.SLIP_END
		return (encoded)
