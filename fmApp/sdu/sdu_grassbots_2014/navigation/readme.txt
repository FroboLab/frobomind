



** Waypoint files

CSV format:
	easting,northing,heading,id,nav_mode,lin_vel,ang_vel,pause,task

easting,northing
	Transverse Mercator coordinates such as Universal Transverse Mercator (UTM)
heading
	Yaw axis orientation measured in the ENU coordinate system (Counter Clock Wise with respect to the Easting (x)-axis
id
	String identifying the route point name
nav_mode
	0=pure pursuit, 1=AB line navigation
lin_vel
	Desired(and maximum) linear velocity
ang_vel
	Maximum angular velocity
pause
	Time to pause after reaching the waypoint [s]. To make the robot stop briefly at the waypoint use a small value such as 0.001
task
	integer32 value published as /fmCommand/implement Only value not allowed is -1000000 


** FroboMind Route Plan server

The protocol used by the FroboMind route plan server  is described at:

http://www.frobomind.org/index.php/People:Kjeld_Jensen:FroboMind_Route_Plan_Server_Protocol


