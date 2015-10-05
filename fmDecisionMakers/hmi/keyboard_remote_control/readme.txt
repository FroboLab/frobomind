README: FroboMind keyboard_remote_control component

Written 2015-10-04 Kjeld Jensen kjeld@frobomind.org


This keyboard_remote_control component provides a standard remote control output
by publishing keyboard input from the user as ROS messages based on the generic
FroboMind RemoteControl.msg topic.

keyboard_remote_control obtains keyboard input by monitoring a topic published
by the FroboMind component keyboard_interface

Using the keyboard for simulating a joystick may not be very intuitive but it
provides an easy opportunity to remote control a mobile robot or simulator
without an external remote control. 


The following key's have been implemented:

a(uto)   Set switch 1 to true (usually enter autonomous behaviour) and neutralize arrow key joystick 

r(emote) Set switch 1 is false (usually enter remote control behavour)

e(nable) Enable deadman signal

Space:   Disable deadman signal, set switch 1 to false and neutralize arrow key joystick

arrow keys are used to step towards max and minimum just like a joystick.

s(top)   Neutralizes the arrow key "joystick"

