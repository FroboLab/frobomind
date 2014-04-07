/****************************************************************************
 # FroboMind
 # Licence and description in .hpp file
 ****************************************************************************/
#include "roboteq_hbl1650/regulator.hpp"

Regulator::Regulator()
{
	p = i = d = i_max = out_max = 0;
	integrator = previous = 0;
}

double Regulator::output_from_input( double setpoint , double input , double period)
{
	// Calculate error
	double error = setpoint - input;

	// Implement sanity checks
	if(period > 0.5)
		period = 0.5;
	if(error > 3.0 | error < -3.0)
		error = 0;


	// Calculate integrator
	integrator += error * period;

	// Implement anti wind up
	if(integrator > i_max)
		integrator = i_max;
	else if(integrator < -i_max)
		integrator = -i_max;

	// Calculate differentiator + sanity check
	double differentiator = ( previous - input ) / period;


	// Calculate output;
	double output = (setpoint*ff) + (error * p) + (integrator * i) + (differentiator * d);

	// Implement output max
	if(output > out_max)
		output = out_max;
	else if(output < -out_max)
		output = -out_max;

	// Upkeep
	previous = input;

	// Debug
	msgs::FloatArrayStamped msg;
	msg.header.stamp = ros::Time::now();
	msg.data.push_back(error);
	msg.data.push_back(output);
	msg.data.push_back(error*p);
	msg.data.push_back(integrator);
	msg.data.push_back(differentiator * d);
	pid_publisher.publish(msg);
	return output;
}
