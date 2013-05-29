/****************************************************************************
 # FroboMind regulator.cpp
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
 ****************************************************************************/
#ifndef REGULATOR_HPP_
#define REGULATOR_HPP_

#include <ros/ros.h>
#include <iostream>
#include <msgs/serial.h>
#include <msgs/IntStamped.h>
#include <msgs/StringStamped.h>


class Regulator
{
	struct gain_t
	{
	  double p;
	  double i;
	  double d;
	};

	struct lr_double_t
	{
	  double left;
	  double right;
	};

	struct lr_int_t
	{
	  double left;
	  double right;
	};

private:
	lr_double_t speed,setpoint,error,previous,integrator,differentiator;
	lr_int_t encoder;

public:
	struct
	{
	  gain_t left;
	  gain_t right;
	} gain;

	double antiwindup;

	Regulator();

	bool get_speeds( double , double , int , int , int * , int * );
	double ticks_to_mps(int);
	int mps_to_ticks(double);
};

#endif /* REGULATOR_HPP_ */
