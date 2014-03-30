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
 # 2014-02-02 Leon: Implemented s-curves
 #
 ****************************************************************************
 # This class implements s-curves for the motorcontroller, tesselating the
 # setpoint into desired setpoints at a given time.
 #
 #             !!!!!!! This is currently untested code !!!!!!!
 # 
 ****************************************************************************/

#ifndef POSITIONGENERATOR_HPP_
#define POSITIONGENERATOR_HPP_

#include <iostream>

class PositionGenerator
{
private:
	struct
	{
		double current, previous;
	} position;

	struct
	{
		double max, current, previous, setpoint, input, tolerance;
	} velocity;

	struct
	{
		double max, current, previous;
	} acceleration;

	struct
	{
		double max;
	} jerk;


    double period, brake_zeroband;

	// Methods to discern state
    bool isBraking(void);
    bool isAtRest(void);
    bool isOnSetpoint(void);
    bool isAccelerating(void);
    bool isDecelerating(void);
    bool isDecliningAcceleration(void);
    bool isIncliningAcceleration(void);

	// Methods performing actions
    void declineAcceleration(void);
    void inclineAcceleration(void);
    void hault(void);
    void enforceVelocityMaximum(void);
    void enforceAccelerationMaximum(void);
    void upkeep(void);
    void brake(void);

	// Math
    int sign(double);

public:
	PositionGenerator();
	~PositionGenerator();

	// Mutators
	void setPeriod(double);
	void setCurrentPosition(double);
	void setCurrentVelocityInput(double);

	void setMaximumVelocity(double);
	void setMaximumAcceleration(double);
	void setMaximumJerk(double);
	void setBrakeZeroband(double);
	void setVelocityTolerance(double);

	// Accessors
	double getNewPosition(void);
};

#endif /* POSITIONGENERATOR_HPP_ */



