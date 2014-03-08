#!/usr/bin/env python

import rospy
from msgs.msg import IntStamped
import serial

serial_device = '/dev/ttyUSB0'
serial_baudrate = 38400
topic_pos = '/fmInformation/encoder_pos'
update_interval = 0.1 # [s]


rospy.init_node('dmm_tech_abs_node')
pub = rospy.Publisher(topic_pos, IntStamped)
msg = IntStamped()

serial_err = False
try:
	ser = serial.Serial(serial_device, serial_baudrate, timeout=1)
except Exception as e:
	serial_err = True
	rospy.logerr (rospy.get_name() + ': Unable to open %s' % (serial_device))

if serial_err == False:
	print 'ok'
	#ser.write (bytes('\0'))  # request position
	ser.write ('\0')

	while not rospy.is_shutdown():
		s = ser.read (1) # read two bytes
		print len(s)
		ser.write ('\0')
		#ser.write (bytes('\0'))  # request position

		msg.data = 4
		pub.publish(msg)
		rospy.sleep(update_interval)

	ser.close()


 

