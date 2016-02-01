This FroboMind component contains a ROS interface for SocketCAN drivers.

Run bin/can_configure.sh to setup the CAN device and bitrate. The default
is can0 and 250000 bps. To change these values please edit the script.

It has been tested succesfully with the USB PEAK CAN adapter.

Please see launch/example.launch for information on launch parameters.

2016-02-01 Kjeld Jensen <kjeld@frobomind.org>

