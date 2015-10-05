#!/usr/bin/env python
#/****************************************************************************
# Frobit wptnav_mission_node
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
#****************************************************************************/
"""
This mission file provides simple mission handling based on input from a user
remote control.

Revision
2013-11-06 KJ First version
2015-03-05 KJ Added queue_size to rospy.Publisher calls (Indigo compatiblity)
2015-03-19 KJ Switched from Bool to BoolStamped messages
2015-09-14 KJ Changed automode message type from BoolStamped to IntStamped
              and default topic name to /fmPlan/automode
2015-09-22 KJ Implemented behaviour classes (remote control and wptnav)
              Inserted HMI abstraction layer through the /fmHMI/remote_control
              and /fmHMI/remote_control_feedback topics.
"""

# ROS import
import rospy
from std_msgs.msg import Bool, Char
from msgs.msg import BoolStamped, IntStamped, RemoteControl, RemoteControlFeedback

# Behaviour import
from behaviour_remote_control import behaviour_remote_control
from behaviour_wptnav import behaviour_wptnav

class mission_node():
	def __init__(self):
		self.update_rate = 20 # [Hz]

		# robot state
		self.BHV_RC = 0
		self.BHV_WPTNAV = 1
		self.bhv = self.BHV_RC

		# load behaviours
		self.bhv_rc = behaviour_remote_control()
		self.bhv_wn = behaviour_wptnav()

		# get topic names
		topic_rc = rospy.get_param("~remote_control_sub",'/fmHMI/remote_control')
		topic_rc_feedback = rospy.get_param("~remote_control_feedback_pub",'/fmHMI/remote_control_feedback')
		deadman_topic = rospy.get_param("~deadman_pub", "/fmSafe/deadman")
		behaviour_topic = rospy.get_param("~automode_pub", "/fmPlan/automode")

		# setup deadman publish topic
		self.deadman_state = False
		self.deadman_msg = BoolStamped()
		self.deadman_pub = rospy.Publisher(deadman_topic, BoolStamped, queue_size=1)

		# setup automode publish topic
		self.behaviour_msg = IntStamped()
		self.behaviour_pub = rospy.Publisher(behaviour_topic, IntStamped, queue_size=1)
		
		# setup subscription topic callbacks
		rospy.Subscriber(topic_rc, RemoteControl, self.on_rc_topic)

		# sall updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_rc_topic(self, msg):
		# get remote control behaviour 
		if (msg.switches & 0x01) == 0:
			behaviour = self.BHV_RC
		else:
			behaviour = self.BHV_WPTNAV

		# if behaviour changed
		if behaviour != self.bhv:
			# suspend current behavour
			if self.bhv == self.BHV_RC:
				self.bhv_rc.suspend()
			elif self.bhv == self.BHV_WPTNAV:
				self.bhv_wn.suspend()
	
			# activate new behaviour
			self.bhv = behaviour
			if self.bhv == self.BHV_RC:
				self.bhv_rc.activate()
				rospy.loginfo(rospy.get_name() + ": Switching to remote control behaviour")
			elif self.bhv == self.BHV_WPTNAV:
				self.bhv_wn.activate()
				rospy.loginfo(rospy.get_name() + ": Switching to waypoint navigation behaviour")


		# pass remote control info to current behaviour
		if self.bhv == self.BHV_RC:
			self.bhv_rc.on_rc_topic(msg)
		elif self.bhv == self.BHV_WPTNAV:
			self.bhv_wn.on_rc_topic(msg)

	def publish_deadman_message(self):
		prev_deadman_state = self.deadman_msg.data

		# retrieve deadman_state from current behaviour
		if self.bhv == self.BHV_RC:
			self.deadman_msg.data = self.bhv_rc.deadman_state
		elif self.bhv == self.BHV_WPTNAV:
			self.deadman_msg.data = self.bhv_wn.deadman_state

		# print a warning if the deadman_state has changed
		if self.deadman_msg.data == True and prev_deadman_state == False:
			rospy.loginfo(rospy.get_name() + ": Deadman signal activated")
		elif self.deadman_msg.data == False and prev_deadman_state == True:
			rospy.loginfo(rospy.get_name() + ": Deadman signal deactivated")

		# publish the deadman_state
		self.deadman_msg.header.stamp = rospy.get_rostime()
		self.deadman_pub.publish (self.deadman_msg)

	def publish_active_behaviour_message(self):
		# publish the current behaviour
		self.behaviour_msg.data = self.bhv
		self.behaviour_msg.header.stamp = rospy.get_rostime()
		self.behaviour_pub.publish (self.behaviour_msg)

	def updater(self):
		while not rospy.is_shutdown():
			# if remote control behaviour
			if self.bhv == self.BHV_RC:
				self.bhv_rc.update()
			# if waypoint navigation behaviour
			elif self.bhv == self.BHV_WPTNAV:
				self.bhv_wn.update()

			# publish messages
			self.publish_deadman_message()
			self.publish_active_behaviour_message()

			# go back to sleep
			self.r.sleep()

# main function.    
if __name__ == '__main__':
    # initialize the node and name it.
    rospy.init_node('wptnav_mission')

    # go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = mission_node()
    except rospy.ROSInterruptException: pass


