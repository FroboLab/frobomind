README
computer_load_monitor information
Written 2014-02-20 Kjeld Jensen, kjeld@frobomind.org
Updated 2016-01-16 Kjeld Jensen, kjeld@frobomind.org


GENERAL INFO
This FroboMind component publishes the computer CPU- and memory load using
the topics:

/fmInformation/cpu_load
/fmInformation/memory_load

The publish frequency is defined by the parameter update_rate supplied to
the component. Default is 5 Hz.


CPU_LOAD
The published value is the cpu load (averaged across all siblings) defined
as the amount of busy time divided by the total time since the last published
value. The value range is [0;1]

The calculation is based on information from /proc/stat where values are
represented in "jiffies" (time intervals defined by USER_HZ which is 100 Hz
on most systems):

http://www.linuxhowtos.org/System/procstat.htm
http://manpages.ubuntu.com/manpages/quantal/man7/time.7.html
http://stackoverflow.com/questions/3017162/how-to-get-total-cpu-usage-in-linux-c

To obtain the USER_HZ value on a linux system run the command:

$ getconf CLK_TCK

For detailed information about the CPU run the command

$ cat /proc/cpuinfo


MEMORY_LOAD
The published value is the memory used currently. The value is based on the 
calculation (MemTotal - MemFree) obtained from the file /proc/meminfo 
The value range is [0;1]

It is the same value displayed before 'used' when running the command:

$ top


SCRIPTS
The scripts directory contains the pythin script export_data.py to export CPU
and Memory load values from a rosbag. Run the script with the filename of a
rosbag as parameter to generate the files rosbag_cpu_load.txt
rosbag_memory_load.txt and rosbag_rosout.txt

Run the script plot_data.py to generate a plot of the CPU and memory load.
The plot is also saved to the file cpu_memory_load.png


