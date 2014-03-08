/*****************************************************************************
# NMEA interface
# Copyright (c) 2011-2014, Søren Hundevadt Nielsen, Leon Bonde Larsen, Mathias Mikkel Neerup, Kjeld Jensen
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
#
# This node reads NMEA messages from a serial stream, decodes the information
# and verifys the checksum.
#
# Created:  2012-xx-xx Leon Larsen
# Modified: 2013-03-04 kjen   bugfix, more robust error handling,
#                             added length to nmea msg, checksum is validated on
#                             incoming msgs regardles of use_nmea_checksum value
#                             OUTGOING CHECKSUM NOT YET IMPLEMENTED
# Modified: 2014-03-07 Mathias & Kjeld, combined NMEA and serial interface
#
#****************************************************************************/

#include "nmea_interface.h"

msgs::nmea nmea_interface::str_to_msg(const std::string& msg){
        //build NMEA message data type
        nmea_msg.header.stamp = ros::Time::now();
        nmea_msg.type.clear();
        nmea_msg.data.clear();
        nmea_msg.length = 0;
        nmea_msg.valid = false;

        if ((msg[0] == '$') && (msg.find(",") != std::string::npos)){
                // test if checksum is appended to the received message
                bool checksum_appended = (msg.find("*") != std::string::npos) ? true : false;

                // Separate into datastring and checksum
                boost::char_separator<char> split("$*\r");
                boost::tokenizer<boost::char_separator<char> > tokens(msg, split);
                std::vector<std::string> nmea_split;
                nmea_split.assign(tokens.begin(), tokens.end());

                // Separate data_string
                boost::char_separator<char> sep("," , "" , boost::keep_empty_tokens);
                boost::tokenizer<boost::char_separator<char> > data_tokens(nmea_split.at(0), sep);
                nmea_msg.data.assign(data_tokens.begin(), data_tokens.end());

                // Extract type and remove type from data. Save length
                nmea_msg.type = nmea_msg.data.at(0);
                nmea_msg.data.erase( nmea_msg.data.begin());
                nmea_msg.length = nmea_msg.data.size();

                // Validate checksum if appended
                if(checksum_appended)
                        nmea_msg.valid = get_checksum( nmea_split.at(0) ) == atox( nmea_split.at(1).c_str() );
        } else {
                ROS_WARN("Unreadable NMEA string discarded");
	}
    return nmea_msg;
}

/* Parses from a string into two hex characters */
int nmea_interface::atox(const char *s)
{
        int ret = 0;
        int cnt = 8;
        while (cnt)
        {
                if ((*s >= '0') && (*s <= '9')) {
                        ret <<= 4;
                        ret += (*s - '0');
                        cnt--;
                        s++;
                        continue;
                }
                if ((*s >= 'a') && (*s <= 'f')) {
                        ret <<= 4;
                        ret += (*s - 'a' + 10);
                        cnt--;
                        s++;
                        continue;
                }
                if ((*s >= 'A') && (*s <= 'F')) {
                        ret <<= 4;
                        ret += (*s - 'A' + 10);
                        cnt--;
                        s++;
                        continue;
                }
                break;
        }
        return ret;
}



std::string nmea_interface::xtoa(unsigned char chk)
{
        std::string str;
        unsigned char nibble;

        /*process high nibble*/
        nibble = (chk >> 4) & 0x0F;
        if(nibble < 10)
                str.push_back( nibble + '0' );
        else
                str.push_back( nibble - 10 + 'A' );

        /*process low nibble*/
        nibble = chk & 0x0F;
        if(nibble < 10)
                str.push_back( nibble + '0' );
        else
                str.push_back( nibble - 10 + 'A' );

        return str;
}

/* Calculates checksum from string */
unsigned char nmea_interface::get_checksum(std::string str)
{
        unsigned char checksum = 0;
        for( int i = 0 ; i < str.size() ; i++)
                checksum ^= str.at(i);
        return checksum;
}



/* Parses from  nmea message to serial message */
msgs::serial nmea_interface::msg_to_str(const msgs::nmea::ConstPtr& msg){
        // Construct message object
        msgs::serial nmea_message;
        nmea_message.header.stamp = msg->header.stamp;

        std::vector<std::string> nmea_vector;

        // Generate nmea string
        nmea_vector.push_back("$");
        nmea_vector.push_back(msg->type);
        nmea_vector.push_back(",");
        nmea_vector.push_back( boost::algorithm::join(msg->data, ",") );

        // Append checksum
        if (use_checksum){
                unsigned char chk = 0;

                // Calculate checksum of type and data fields
                for(unsigned int i = 1 ; i < 4 ; i++)
                        chk ^= get_checksum( nmea_vector.at(i) );

                // Append to vector
                nmea_vector.push_back("*");
                nmea_vector.push_back( xtoa(chk) );
        }

        // Append string end and publish combined message
        nmea_vector.push_back("\r\n");
        nmea_message.data = boost::algorithm::join(nmea_vector,"");
	return nmea_message;
}


/* setter to checksum */
void nmea_interface::set_use_checksum(bool checksum){
	use_checksum = checksum;
}
