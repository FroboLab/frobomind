Pose 2D Estimator - robot drive simulation

Copyright (c) 2013, Kjeld Jensen <kjeld@frobomind.org> 
All rights reserved.

It is licensed using the BSD 3-Clause license. For detailed information
please see the source files and the bottom of this file. If you find
this software useful please let me know.


==== QUICK START ====

The simulation directory contains files that allow testing the pose_2d_estimator
in a simulated environment without ROS. 

Edit the file  generate_simulation_data.launch to load the rosbag you want to work with.
Then run:

	roslaunch generate_simulation_data.launch

to once make sure that all sensor data are available for the simulation. You will
have to quit manually when the rosbag doesn't publish more messages. Then run:

	export_simulation_bag.py

once to export rosbag data to a suitable comma delimited format. Now you may run 

	run_simulation.sh

to perform the actual simulation.


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


