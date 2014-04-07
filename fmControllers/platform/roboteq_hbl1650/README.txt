RoboTeQ HBL1650 driver for ROS, FroboMind 

Notice that the controller is installed as a library. 
To see instantiation examples see for example fmLib/platform_support/sdu_armadillo3

The controller has two modes as either velocity control, where the regulator is applied 
directly to the velocity setpoint or position control, where the velocity setpoint 
is converted to a position before being fed to the regulator.

The RoboTeQ driver consists of 3 main classes

- The RoboTeQ class
    Implements the low level communicaion with the controller
    Holds methods for sending and receiving strings
    Holds virtual methods to allow callbacks from derived classes

- The hbl1650 class 
    Is derived from the RoboTeQ class 
    Implements the specific controller type.
    Instantiates the channel object
    Is essentially an interface class to bind the RoboTeQ and channel classes.
    Handles communication with the ROS core
    Handles setup of the controller

- The Channel class 
    Implements the channel object
    Holds callback methods for feedback
    Holds timer callback for regulating
    Instantiates the regulator and the position generator
    Can run either position control or velocity control

- The Regulator class
    Implements a general PID controller
    Is used in both position control and velocity control

- The PositionGenerator class
    Implements s-curves for the position control
    Given a desired setpoint it converts to a corrected setpoint
    The corrected setpoint implements constant jerk s-curves
    A simulation of the S-curve generator can be found in the scripts folder
    To run:

	python s_curve_demo.py

    A detailed description of the simulation can be found in the sourcecode
