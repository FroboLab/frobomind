#!/usr/bin/env python
#*****************************************************************************
# Copyright (c) 2013, Leon Bonde Larsen <leon@frobomind.org>
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
import rospy
from msgs.msg import serial, BoolStamped, IntStamped, StringStamped, gpgga

class FaultDetector():
    def __init__(self):        
        # Init time stamps
        self.last_validated_imu = rospy.Time.now()
        self.last_validated_gps = rospy.Time.now()
        self.last_validated_left_hall = rospy.Time.now()
        self.last_validated_right_hall = rospy.Time.now()
        self.last_validated_status = rospy.Time.now() 
        self.last_validated_gpgga = rospy.Time.now() 

        # Init settings
        self.max_time_sec = 0.5
        self.period = 1
        self.max_time = rospy.Duration(self.max_time_sec)
        
        # Init messages
        self.all_ok_message = BoolStamped()
        
        # Init subscribers
        self.imu_sub = rospy.Subscriber('/fmData/imu_rx', serial, self.onImu )
        self.gps_sub = rospy.Subscriber('/fmData/gps_rx', serial, self.onGps )
        self.hall_left_sub = rospy.Subscriber('/fmInformation/encoder_left', IntStamped, self.onLeftHall )
        self.hall_right_sub = rospy.Subscriber('/fmInformation/encoder_right', IntStamped, self.onRightHall )
        self.status_sub = rospy.Subscriber('/fmData/status', StringStamped, self.onStatus )
        self.gpgga_sub = rospy.Subscriber('/fmInformation/gpgga', gpgga, self.onGpgga )
        
        # Init publisher
        self.all_ok_pub = rospy.Publisher('/fmSafety/all_ok', BoolStamped)
        
        # Init Timer
        rospy.Timer(rospy.Duration(self.period), self.onTimer)          

    def onStatus(self,msg):       
        if "under voltage" in msg.data :
            self.last_validated_status = rospy.Time.now()
                
    def onImu(self,msg):       
        if "VNQMR" in msg.data :
            self.last_validated_imu = rospy.Time.now()
            
    def onGps(self,msg):       
        if "GPGGA" in msg.data :
            self.last_validated_gps = rospy.Time.now()
    
    def onLeftHall(self,msg):       
        self.last_validated_left_hall = rospy.Time.now()
        
    def onRightHall(self,msg):       
        self.last_validated_right_hall = rospy.Time.now()
        
    def onGpgga(self,msg):       
        if msg.fix == 4 :
            self.last_validated_gpgga = rospy.Time.now()
            
    def onTimer(self, event):
        now = rospy.Time.now()
        all_ok = True
        if(now - self.last_validated_imu) > self.max_time :
            rospy.logwarn(rospy.get_name() + ": IMU error!!!")
            all_ok = False
        if(now - self.last_validated_gps) > self.max_time :
            rospy.logwarn(rospy.get_name() + ": GPS error!!!")
            all_ok = False
        if(now - self.last_validated_left_hall) > self.max_time :
            rospy.logwarn(rospy.get_name() + ": Left hall error!!!")
            all_ok = False
        if(now - self.last_validated_right_hall) > self.max_time :
            rospy.logwarn(rospy.get_name() + ": Right hall error!!!")
            all_ok = False
        if(now - self.last_validated_right_hall) > self.max_time :
            rospy.logwarn(rospy.get_name() + ": Status error!!!")
            all_ok = False
        if(now - self.last_validated_gpgga) > self.max_time :
            rospy.logwarn(rospy.get_name() + ": GPS has no fix!!!")
            all_ok = False
        
        
        # Publish all_OK                   
        self.all_ok_message.header.stamp = rospy.Time.now()
        self.all_ok_message.data = all_ok
        self.all_ok_pub.publish(self.all_ok_message)
         
if __name__ == '__main__':
    rospy.init_node('fault_detector_node')
    imu = FaultDetector()
    rospy.spin()

