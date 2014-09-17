#!/usr/bin/env python
#/****************************************************************************
# Waypoint Navigation: Globals
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

# internal waypoint list structure
W_E = 0 
W_N = 1
W_HEADING = 2
W_ID = 3
W_NAV_MODE = 4
W_LIN_VEL = 5
W_ANG_VEL = 6
W_PAUSE = 7
W_TASK = 8

# csv file waypoint list structure
CSV_E = 0 
CSV_N = 1
CSV_HEADING = 2
CSV_ID = 3
CSV_NAV_MODE = 4
CSV_LIN_VEL = 5
CSV_ANG_VEL = 6
CSV_PAUSE = 7
CSV_TASK = 8

# route point command
ROUTEPT_CMD_DELETE = 0
ROUTEPT_CMD_DELETE_THEN_APPEND = 1
ROUTEPT_CMD_APPEND = 2

# route point navigation mode
ROUTEPT_NAV_MODE_PP = 0 # pure pursuit
ROUTEPT_NAV_MODE_AB = 1 # AB line navigation

ROUTEPT_INVALID_DATA = -1000000

# HMI defines
HMI_ID_DEADMAN = 0
HMI_ID_MODE = 1
HMI_ID_GOTO_WAYPOINT = 2

HMI_MODE_MANUAL = 0
HMI_MODE_AUTO = 1

