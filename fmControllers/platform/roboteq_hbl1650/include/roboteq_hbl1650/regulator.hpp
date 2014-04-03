/****************************************************************************
 # FroboMind
 # Copyright (c) 2011-2013, author Leon Bonde Larsen <leon@bondelarsen.dk>
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 #	* Redistributions of source code must retain the above copyright
 #  	notice, this list of conditions and the following disclaimer.
 #	* Redistributions in binary form must reproduce the above copyright
 #  	notice, this list of conditions and the following disclaimer in the
 #  	documentation and/or other materials provided with the distribution.
 #	* Neither the name FroboMind nor the
 #  	names of its contributors may be used to endorse or promote products
 #  	derived from this software without specific prior written permission.
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
 ****************************************************************************
 # 2013-06-02 Leon: Implemented regulator
 #
 ****************************************************************************
 # This class implements a general PID regulator as the method output_from_input
 # taking as input the current setpoint, feedback and time since last call.
 ****************************************************************************/
#ifndef REGULATOR_HPP_
#define REGULATOR_HPP_

#include <ros/ros.h>
#include <msgs/FloatArrayStamped.h>
#include <iostream>

class Regulator
{
/*
 * Class implementing the concept of a PID controller simplified to work on any
 * numbers of equal units
 * */

private:
	double previous,integrator;
	ros::Publisher pid_publisher;

public:
	double p,i,d,ff,i_max,out_max;

	Regulator();

	void setPidPub(ros::Publisher pub){pid_publisher = pub;}
	double output_from_input( double , double , double);
	void reset_integrator(){integrator = 0;}
	void set_params( double p_gain , double i_gain , double d_gain , double ff_gain, double imax , double outmax)
	{p = p_gain; i = i_gain; d = d_gain; ff = ff_gain; i_max = imax; out_max = outmax; }
};

#endif /* REGULATOR_HPP_ */
