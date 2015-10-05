README: FroboMind wiimote_remote_control component

Written 2015-10-04 Kjeld Jensen kjeld@frobomind.org

This wiimote_remote_control component provides a standard remote control output
by publishing input from a Nintendo Wiimote as ROS messages based on the generic
FroboMind RemoteControl.msg topic.

wiimote_remote_control obtains input from the Wiimote by monitoring a topic
published the generic ROS package wiimote


The following mapping between the Wiimote and RemoteControl.msg has been implemented:

A        Flip the switch 1 (usually switches between remote control and autonomous behaviour)
B        Enable deadman signal

The four arrow keys updates digital joystick A of RemoteControl.msg

Tilting the wiimote will output to analog joystick A of RemoteControl.msg

All other push buttons on the Wiimote update the buttons var of RemoteControl.msg



The following features have not yet been implemented:

Feedback using RemoteControlFeedback.msg to the four LED's and rumble function.
This is available in the ROS package wiimote

Wiimote battery voltage, this is not available through the ROS package wiimote
but it should be available from the linux joystick device.


Please notice that the current version of the generic ROS package wiimote
outputs a number of warnings when the Wiimote connects. The reason is that
the file wiimote_node.py is not properly updated for ROS Indigo which
requires that all publishers must define a queue_size. This may be mitigated
by editing the file /opt/ros/indigo/lib/wiimote/wiimote_node.py adding 
queue_size=1 to all declarations of publishers.

