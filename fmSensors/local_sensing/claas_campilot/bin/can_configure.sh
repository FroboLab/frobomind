#!/bin/sh
#
# Script for setting up a CAN device
# If real devices are available they are used - if not fake ones are created.
#
# Original script by Mathias Mikkel Neerup manee12@student.sdu.dk
# Ported to FroboMind by Kjeld Jensen kjeld@frobomind.org
#

setup_can() {
	echo "Configuring $1 for $2 bps"
	# Try to down the interface - if it fails the interface does not exist.
	sudo ifconfig $1 down
	if [ $? -ne 0 ]; then
		sudo ip link add name $1 type vcan
	fi

	# Setup flags - virtual devices do not support these
	sudo ip link set $1 type can restart-ms 100
	sudo ip link set $1 up type can bitrate $2 
	if [ $? -eq 0 ]; then
		echo "Ok"
	else
		echo "$1 configured as a virtual device"
	fi
	sudo ifconfig $1 up
	if [ $? -ne 0 ]; then
		echo "Failed bringing up $1"
	fi

}

CAN_DEVICE=can0
CAN_BITRATE=250000
setup_can $CAN_DEVICE $CAN_BITRATE

