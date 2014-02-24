
HISTORY
2014-02-24 Kjeld Jensen <kjeld@frobomind.org> First release.


GENERAL INFO
This file contains documentation for the FroboMind plot_pid component

Please look at the file plot_pid_example.launch for an example on how to launch plot_pid_node.py

Plot PID subscribes to a message containing updated values from the PID controller. The message is described below:

Type: FloatArrayStamped 
	Header
	float64[] data

data contains the following value:
	error input
	PID output (total)
	P contribution
	I contribution
	D contribution
	Feed forward contribution (optional)


