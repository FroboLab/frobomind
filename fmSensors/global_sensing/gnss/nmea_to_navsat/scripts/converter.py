#!/usr/bin/env python
#/****************************************************************************
# FroboMind NMEA to NavSat Converter
# Copyright (c) 2011-2013, author Leon Bonde Larsen <leon@bondelarsen.dk>
# Some parts of this code is copied from 
# nmea_gps_driver Copyright (c) 2012, Steven Martin, Eric Perko
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
import rospy, string, math, time, calendar
from sensor_msgs.msg import NavSatFix,NavSatStatus,TimeReference
from geometry_msgs.msg import TwistStamped
from msgs.msg import nmea

class converter():
    """    
        Converter class
    """
    def __init__(self):
        rospy.init_node('nmea_to_navsat_converter')     
        self.gpspub = rospy.Publisher('fix', NavSatFix)
        self.gpsVelPub = rospy.Publisher('vel',TwistStamped)
        self.gpstimePub = rospy.Publisher('time_reference', TimeReference)
        self.nmea_topic = rospy.get_param("~nmea_topic",'/fmLib/nmea_from_gps')
        self.status_sub = rospy.Subscriber(self.nmea_topic, nmea , self.onNmea)
        
        self.frame_id = rospy.get_param('~frame_id','gps')
        if self.frame_id[0] != "/":
            self.frame_id = self.addTFPrefix(self.frame_id)
        self.time_ref_source = rospy.get_param('~time_ref_source', self.frame_id)
        self.useRMC = rospy.get_param('~useRMC', False)
        #useRMC == True -> generate info from RMC+GSA
        #useRMC == False -> generate info from GGA
        
        self.navData = NavSatFix()
        self.gpsVel = TwistStamped()
        self.gpstime = TimeReference()
        
        self.gpstime.source = self.time_ref_source
        self.navData.header.frame_id = self.frame_id
        self.gpsVel.header.frame_id = self.frame_id
        self.GPSLock = False
        
    #nmea_utc should be a string of form hhmmss
    def convertNMEATimeToROS(self,nmea_utc):
        #Get current time in UTC for date information
        utc_struct = time.gmtime() #immutable, so cannot modify this one
        utc_list = list(utc_struct)
        hours = int(nmea_utc[0:2])
        minutes = int(nmea_utc[2:4])
        seconds = int(nmea_utc[4:6])
        utc_list[3] = hours
        utc_list[4] = minutes
        utc_list[5] = seconds
        unix_time = calendar.timegm(tuple(utc_list))
        return rospy.Time.from_sec(unix_time)

    #Add the tf_prefix to the given frame id
    def addTFPrefix(self,frame_id):
        prefix = ""
        prefix_param = rospy.search_param("tf_prefix")
        if prefix_param:
            prefix = rospy.get_param(prefix_param)
            if prefix[0] != "/":
                prefix = "/%s" % prefix
    
        return "%s/%s" % (prefix, frame_id)

    def onNmea(self,msg):
        timeNow = rospy.get_rostime()
        
        try:
            if self.useRMC:
                #Check for satellite lock
                if 'GSA' in msg.type:
                    lockState = int(msg.data[1])
                    #print 'lockState=',lockState
                    if lockState == 3:
                        self.GPSLock = True
                    else:
                        self.GPSLock = False
                #if not satellite lock message parse it separately
                else:
                    if self.GPSLock == True:
                        if 'RMC' in msg.type:
                            #print msg.data
                            self.gpsVel.header.stamp = timeNow
                            self.gpsVel.twist.linear.x = float(msg.data[6])*0.514444444444*math.sin(float(msg.data[7]))
                            self.gpsVel.twist.linear.y = float(msg.data[6])*0.514444444444*math.cos(float(msg.data[7]))
                            self.gpsVelPub.publish(self.gpsVel)

                            self.navData.status.status = NavSatStatus.STATUS_FIX
                            self.navData.header.stamp = gpsVel.header.stamp
                            self.navData.status.service = NavSatStatus.SERVICE_GPS

                            self.gpstime.header.stamp = self.gpsVel.header.stamp
                            self.gpstime.time_ref = self.convertNMEATimeToROS(msg.data[0])

                            longitude = float(msg.data[4][0:3]) + float(msg.data[4][3:])/60
                            if msg.data[5] == 'W':
                                longitude = -longitude

                            latitude = float(msg.data[2][0:2]) + float(msg.data[2][2:])/60
                            if msg.data[3] == 'S':
                                latitude = -latitude

                            #publish data
                            self.navData.latitude = latitude
                            self.navData.longitude = longitude
                            self.navData.altitude = float('NaN')
                            self.navData.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                            self.gpspub.publish(self.navData)
                            self.gpstimePub.publish(self.gpstime)
                    else:
                        pass
                        #print data
            else:
                #Use GGA
                #No /vel output from just GGA
                if 'GGA' in msg.type:
                    gps_quality = int(msg.data[5])
                    if gps_quality == 0:
                        self.navData.status.status = NavSatStatus.STATUS_NO_FIX
                    elif gps_quality == 1:
                        self.navData.status.status = NavSatStatus.STATUS_FIX
                    elif gps_quality == 2:
                        self.navData.status.status = NavSatStatus.STATUS_SBAS_FIX
                    elif gps_quality in (4,5):
                        #Maybe 2 should also sometimes be GBAS... but pretty
                        #sure RTK has to have a base station
                        self.navData.status.status = NavSatStatus.STATUS_GBAS_FIX
                    else:
                        self.navData.status.status = NavSatStatus.STATUS_NO_FIX
                    self.navData.status.service = NavSatStatus.SERVICE_GPS

                    self.navData.header.stamp = timeNow

                    latitude = float(msg.data[1][0:2]) + float(msg.data[1][2:])/60
                    if msg.data[2] == 'S':
                        latitude = -latitude
                    self.navData.latitude = latitude

                    longitude = float(msg.data[3][0:3]) + float(msg.data[3][3:])/60
                    if msg.data[4] == 'W':
                        longitude = -longitude
                    self.navData.longitude = longitude

                    hdop = float(msg.data[7])
                    self.navData.position_covariance[0] = hdop**2
                    self.navData.position_covariance[4] = hdop**2
                    self.navData.position_covariance[8] = (2*hdop)**2 #FIX ME
                    self.navData.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

                    #Altitude is above ellipsoid, so adjust for mean-sea-level
                    altitude = float(msg.data[8]) + float(msg.data[10])
                    self.navData.altitude = altitude


                    self.gpstime.header.stamp = timeNow
                    self.gpstime.time_ref = self.convertNMEATimeToROS(msg.data[0])

                    self.gpspub.publish(self.navData)
                    self.gpstimePub.publish(self.gpstime)
        except ValueError as e:
            rospy.logwarn("Value error, likely due to missing fields in the NMEA messages. Error was: %s" % e)

if __name__ == '__main__':
    try:
        node = converter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
