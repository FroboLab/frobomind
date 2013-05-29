#include "roboteq/roboteq.hpp"

Regulator::Regulator()
{
	gain.left.p = 0;
	gain.left.i = 0;
	gain.left.d = 0;
	gain.right.p = 0;
	gain.right.i = 0;
	gain.right.d = 0;
	antiwindup = 0;
}

bool Regulator::get_speeds( double sp_left , double sp_right , int enc_left , int enc_right, int *speed_left , int *speed_right )
{


	setpoint.left = sp_left;
	setpoint.right = sp_right;

	error.left = setpoint.left - ticks_to_mps(enc_left);
	error.right = setpoint.right - ticks_to_mps(enc_right);

	integrator.left += error.left;
	integrator.right += error.right;

	if(integrator.left > antiwindup)
		integrator.left = antiwindup;
	else if(integrator.left < -antiwindup)
		integrator.left = -antiwindup;
	if(integrator.left > antiwindup)
		integrator.left = antiwindup;
	else if(integrator.left < -antiwindup)
		integrator.left = -antiwindup;

	if(integrator.right > antiwindup)
		integrator.right = antiwindup;
	else if(integrator.right < -antiwindup)
		integrator.right = -antiwindup;
	if(integrator.right > antiwindup)
		integrator.right = antiwindup;
	else if(integrator.right < -antiwindup)
		integrator.right = -antiwindup;

	differentiator.left = ticks_to_mps(previous.left - error.left);
	differentiator.right = ticks_to_mps(previous.right - error.right);

	speed.left = mps_to_ticks( error.left * gain.left.p + integrator.left * gain.left.i * period + differentiator.left * gain.left.d / period );
	speed.right = mps_to_ticks( error.right * gain.right.p + integrator.right * gain.right.i + differentiator.right * gain.right.d );

	return true;
}
