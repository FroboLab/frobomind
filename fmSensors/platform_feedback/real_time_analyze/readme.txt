
The purpose of this FroboMind real_time_analyze component is to analyze the
realtime performance of a computer running FroboMind. 

*** How it works ***

The real_time_analyze_node.py runs at a rate of 100 Hz. At each update it sends
one '$' char to a serial port.

The serial port is connected to an Atmel ATmega microcontroller which counts
the number of 100 ns intervals between each received char. This number is
 transmitted as a single byte to the serial port immediately after receiving
the '$' char.

If the number of 100 ns intervals exceed 250 (corresponding to 25 ms) the
number 251 is transmitted instead.

The real_time_analyze_node.py receives these numbers and publish them on a
topic named /fmInformation/rt_timing


*** Instructions ***

1. Program an Atmel ATmega48 based RoboCard (http://www.robocard.dk), Arduino
or a similar microcontroller board with the firmware available in the
/firmware directory.

2. Connect the microcontroller to a serial port on the PC running FroboMind

3. Include the real_time_analyze_node.py in your launch file as examplified
in /launch/real_time_analyze.launch

4. Run FroboMind and save a rosbag.

5. Use the export_data.py script under /scripts to export the real time timing
data from the rosbag.

6. Use the plot_data.py script under /scripts to visualize the result.


The firmware may easily be ported to ATmega's in the same family such as
ATmega88, ATmega168 and ATmega328. Just change the settings in the
/firmware/Makefile file.



Written 2014-06-21 by Kjeld Jensen <kjeld@frobomind.org>

