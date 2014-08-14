Ublox configuration loader (UbloxCFG)

Copyright (c) 2013, Kjeld Jensen <kjeld@frobomind.org> 
All rights reserved.

It is licensed using the BSD 3-Clause license. For detailed information
please see the source files and the bottom of this file. If you find
this software useful please let me know.


==== QUICK START ====

Compile UbloxCFG by running

	make

Run this command to load the configuration to the Ublox GPS. Remember
to modify device and file name according to your setup.

	ubloxcfg /dev/ttyACM0 9600 yourconfigfile.txt


==== INTRODUCTION ====

Some Ublox GPS modules do not have a built-in flash and therefore need
to be configured after each power up. This is possible using the
Ublox u-center software but it only runs on Windows and requires user
interaction.

UbloxCFG is a small utility that sends the content of a configuration
file to a Ublox GPS. Compilation has been verified under Ubuntu 12.04
as well as an old Debian Etch installation.

Use the Ublox u-center software to create a configuration file for
your setup. This is a text file where each line corresponds to a
setting in u-center.

You may remove any line from the text file that does not begin with
CFG- as it is disregarded anyway. In my experience it is a good idea
to remove the CFG-USB line unless you need this as it causes the
Ublox GPS to stop responding for a short while.


==== DEVELOPMENT ====

The Ublox UBX protocol is described in detail in the document
"u-blox 6 Receiver Description Including Protocol Specification"
which is available at www.ublox.com


==== LICENSE ====

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name FroboMind nor the
     names of its contributors may be used to endorse or promote products
     derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


