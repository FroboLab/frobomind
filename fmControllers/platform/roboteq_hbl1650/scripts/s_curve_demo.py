#!/usr/bin/env python
'''
Created on Jan 29, 2014

@author: Leon Bonde Larsen

This is a quick test of the algorithm developed to generate s-curves in a system
with varying setpoints. Most implementations are based on systems where the entire
curve can be calculated on beforehand, but this is not possible in a system where
the setpoint varies over time.

The actual ode implemented in C++ is an exact copy of the code running in this 
simulation.

Being just an internal test, the settings are placed in the init method on the
Controller class. The controller is based on the constant jerk principle leading
to linear accelerations and second order velocities.

In the current test, the initial velocity is -1m/s and the user setpoint is +2m/s.
What is seen in the plot is that the system first brakes with  ramp to velocity 0m/S
and then accelerates and decelerates to hit the 2m/s velocity.
'''
from matplotlib import pyplot as plt
from control import Controller

plt.figure(num=None, figsize=(10, 6), dpi=80, facecolor='w', edgecolor='k')
ctrl = Controller()
           
ctrl.simulate()

plt.plot(ctrl.time,ctrl.pos)
plt.plot(ctrl.time,ctrl.vel)
plt.plot(ctrl.time,ctrl.acc)
plt.legend(["position","velocity","acceleration"])
plt.show()