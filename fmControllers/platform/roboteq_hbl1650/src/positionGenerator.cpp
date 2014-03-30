/****************************************************************************
 # FroboMind
 # Licence and description in .hpp file
 ****************************************************************************/
#include "roboteq_hbl1650/positionGenerator.hpp"

PositionGenerator::PositionGenerator()
{

//	position.current = position.previous = velocity.max = velocity.current = velocity.previous = velocity.setpoint =
//			velocity.input = velocity.tolerance = acceleration.max = acceleration.current = acceleration.previous =
//			jerk.max = period = brake_zeroband = 0;
	position.current = 0.0;
	position.previous = 0.0;
	velocity.max = 0.0;
	velocity.current = 0.0;
	velocity.previous = 0.0;
	velocity.setpoint = 0.0;
	velocity.input = 0.0;
	velocity.tolerance = 0.0;
	acceleration.max = 0.0;
	acceleration.current = 0.0;
	acceleration.previous = 0.0;
	jerk.max = 0.0;
	period = 0.0;
	brake_zeroband = 0.0;
}

PositionGenerator::~PositionGenerator()
{

}

void PositionGenerator::setMaximumVelocity(double max)
{
	velocity.max = max;
	std::cout << "Maximum velocity has been set to " << velocity.max << std::endl;
}

void PositionGenerator::setMaximumAcceleration(double max)
{
	acceleration.max = max;
	std::cout << "Maximum acceleration has been set to " << acceleration.max << std::endl;
}

void PositionGenerator::setMaximumJerk(double max)
{
	jerk.max = max;
	std::cout << "Maximum jerk has been set to " << jerk.max << std::endl;
}

void PositionGenerator::setBrakeZeroband(double zb)
{
	brake_zeroband = zb;
	std::cout << "Velocity zeroband has been set to " << brake_zeroband << std::endl;
}

void PositionGenerator::setVelocityTolerance(double tol)
{
	velocity.tolerance = tol;
	std::cout << "Velocity tolerance has been set to " << velocity.tolerance << std::endl;
}

void PositionGenerator::setPeriod(double per)
{
	period = per;
}

void PositionGenerator::setCurrentPosition(double pos)
{
	position.previous = pos;
}

void PositionGenerator::setCurrentVelocityInput(double sp)
{
	velocity.input = sp;
}

bool PositionGenerator::isBraking(void)
{
	return (velocity.setpoint == 0) || (velocity.setpoint > 0 && velocity.previous < 0) || (velocity.setpoint < 0 && velocity.previous > 0);
}

bool PositionGenerator::isAtRest(void)
{
	return velocity.setpoint == 0 && (velocity.previous < velocity.tolerance) && (velocity.previous >- velocity.tolerance);
}

bool PositionGenerator::isOnSetpoint(void)
{
	return (velocity.previous < velocity.setpoint + velocity.tolerance) && (velocity.previous > velocity.setpoint - velocity.tolerance);
}

bool PositionGenerator::isAccelerating(void)
{
	return velocity.setpoint > velocity.previous;
}

bool PositionGenerator::isDecelerating(void)
{
	return velocity.setpoint < velocity.previous;
}

bool PositionGenerator::isDecliningAcceleration(void)
{
	return velocity.previous > velocity.setpoint/2;
}

bool PositionGenerator::isIncliningAcceleration(void)
{
	return velocity.previous < velocity.setpoint/2;
}

void PositionGenerator::declineAcceleration(void)
{
    acceleration.current = acceleration.previous - jerk.max*period;
    enforceAccelerationMaximum();
    velocity.current = velocity.previous + acceleration.current*period;
}

void PositionGenerator::inclineAcceleration(void)
{
    acceleration.current = acceleration.previous + jerk.max*period;
    enforceAccelerationMaximum();
    velocity.current = velocity.previous + acceleration.current*period;
}

void PositionGenerator::enforceAccelerationMaximum(void)
{
    if (acceleration.current < -acceleration.max)
    	acceleration.current = -acceleration.max;
    else if (acceleration.current > acceleration.max)
    	acceleration.current = acceleration.max;
    else if (sign(acceleration.current)*sign(acceleration.previous) < 0 )
        acceleration.current = acceleration.previous; //prohibit change of sign
}

int PositionGenerator::sign(double number)
{
	if(number < 0)
		return -1;
	else
		return 1;
}

void PositionGenerator::enforceVelocityMaximum(void)
{
    if (velocity.current < -velocity.max)
    	velocity.current = -velocity.max;
    else if (velocity.current > velocity.max)
    	velocity.current = velocity.max;
}

void PositionGenerator::upkeep(void)
{
    position.previous = position.current;
    velocity.previous = velocity.current;
    acceleration.previous = acceleration.current;
}

void PositionGenerator::hault(void)
{
    acceleration.current = 0;
    velocity.current = velocity.previous;
}

void PositionGenerator::brake(void)
{
    if (velocity.previous > brake_zeroband)
    {
        declineAcceleration();
    }
    else if (velocity.previous < -brake_zeroband)
    {
        inclineAcceleration();
    }
    else
    {
        if (velocity.previous > velocity.tolerance)
        {
            if (acceleration.previous >= 0)
            {
                acceleration.current = -acceleration.max/2;
                velocity.current = velocity.previous + (acceleration.current*period);
            }
            else
            {
                inclineAcceleration();
            }
        }
        else if (velocity.previous < -velocity.tolerance )
        {
            if (acceleration.previous <= 0)
            {
                acceleration.current = acceleration.max/2;
                velocity.current = velocity.previous + (acceleration.current*period);
            }
            else
            {
                declineAcceleration();
            }
        }
        else
        {
            if (velocity.input > velocity.tolerance)
            {
                inclineAcceleration();
            }
            else if (velocity.input < -velocity.tolerance)
            {
                declineAcceleration();
            }
            else
            {
                acceleration.current = 0;
                velocity.current = 0;
            }
        }
    }
}

double PositionGenerator::getNewPosition(void)
{
//    std::cout << "Position generator pre-state:  Position: "<< position.current << " velocity: " << velocity.current
//    		<< " acceleration: " << acceleration.current << " setpoint:" << velocity.setpoint << std::endl;

    if (velocity.input > velocity.max)
        velocity.setpoint = velocity.max;
    else if (velocity.input < -velocity.max)
        velocity.setpoint = -velocity.max;
    else
        velocity.setpoint = velocity.input;


    if ( isBraking() )
    {
    	// Velocity setpoint is zero(velocity.current*period);// or the robot is changing direction
        velocity.setpoint = 0;
        brake();
        std::cout << "Position generator is in braking mode" << std::endl;
    }
    else if ( isAtRest() )
    {
    	// Velocity setpoint is zero and the robot is at rest
        hault();
        std::cout << "Position generator is in hault mode" << std::endl;
    }
    else if ( isOnSetpoint() )
    {
    	// The current velocity is within tolerances of the setpoint
        acceleration.current = 0;
        velocity.current = velocity.previous;
        std::cout << "Position generator is in on-setpoint mode" << std::endl;
    }
    else if ( isAccelerating() )
    {
    	// The current velocity is smaller than setpoint
    	std::cout << "Position generator is in acceleration mode";
        if ( isDecliningAcceleration() )
        {
            declineAcceleration();
            std::cout << " and is declining acceleration rate";
        }
        else if ( isIncliningAcceleration() )
        {
            inclineAcceleration();
            std::cout << " and is inclining acceleration rate";
        }
        else
        {
            acceleration.current = acceleration.max;
            velocity.current = velocity.previous + (acceleration.current*period);
            std::cout << " and is at max acceleration rate";
        }
    }
    else if ( isDecelerating() )
    {
    	// The current velocity is smaller than setpoint
    	std::cout << "Position generator is in deceleration mode";
        if ( isDecliningAcceleration() )
        {
            declineAcceleration();
            std::cout << " and is declining acceleration rate";
        }
        else if ( isIncliningAcceleration() )
        {
            inclineAcceleration();
            std::cout << " and is inclining acceleration rate";
        }
        else
        {
            acceleration.current = -acceleration.max;
            velocity.current = velocity.previous + (acceleration.current*period);
            std::cout << " and is at max acceleration rate";
        }
    }
    enforceVelocityMaximum();
    position.current = position.previous + (velocity.current*period);
    upkeep();
    std::cout << std::endl;
    std::cout << "Position generator post-state:  Position: "<< position.current << " velocity: " << velocity.current
    		<< " acceleration: " << acceleration.current << " setpoint:" << velocity.setpoint << std::endl;
    return position.current;
}
