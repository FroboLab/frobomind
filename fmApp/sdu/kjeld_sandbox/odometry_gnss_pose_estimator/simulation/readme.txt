Pose Estimator - robot drive simulation

Copyright (c) 2013-2014, Kjeld Jensen <kjeld@frobomind.org> 
All rights reserved.

It is licensed using the BSD 3-Clause license. For detailed information
please see the source files and the bottom of this file. If you find
this software useful please let me know.


==== QUICK START ====

OBS OBS OBS
This simulation directory is inherited from the FroboMind pose_2d_estimator
component. It has been tentatively fitted to this pose estimator but the
simulator output is not yet 100% comparable to the odometry_gnss_pose_estimator
node.



Run:

	export_data_from_bag.py

once to export rosbag data to a suitable comma delimited format. If you have
used the replay.launch script to generated simulated odometry and pose estimates
you have to run this instead:

	export_sim_data_from_bag.py

Now you may run 

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
   * Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

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


