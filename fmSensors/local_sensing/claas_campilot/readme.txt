Updated 2016-03-08 Kjeld Jensen kjen@mmmi.sdu.dk


## general notes

The launch file launch/claas_campilot_example.launch assumes that a CAN
adapter using the can_socketcan interface driver is used. The script
bin/can_configure.sh is only relevant when using this interface driver. 



## CLAAS campilot camera topics

can_from_device_sub		"/fmSignal/can_from_campilot"
can_to_device_pub		"/fmSignal/can_to_campilot"

machine_auto_mode_sub		"/fmPlan/cam_machine_automode"
	set to true when well aligned in row

machine_velocity_sub		"/fmKnowledge/cam_machine_velocity"
	update with the machine velocity [mm/s]

cam_rows_pub			"/fmKnowledge/cam_rows"
	row angle [1/100 degrees], distance [1/10 mm], quality factor [0;255]



## CLAAS campilot camera parameters

cam_program			0x03 (single plant row program)
cam_height			160 [cm] from ground surface to center of camera
cam_angle			45 [degrees] tilt with respct to vertical
cam_threshold			0x80 unused...
cam_target_width		15 [cm]
cam_target_height		12 [cm]
cam_target_distance		75 [cm] inter-row distance
cam_minor_distance		75 [cm] should be the same as cam_target_distance
cam_number_of_rows		3
cam_rows_between_wheels		3 should be equal to cam_rows_between_wheels


