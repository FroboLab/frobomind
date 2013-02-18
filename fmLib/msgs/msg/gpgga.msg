Header header
time time_recv		# Data received time stamp 
string time		# [UTC] Retrieved from the GPS data
float64 lat		# [-90;90 deg] WGS-84 latitude in decimal degrees
float64 lon		# [-180;180 deg] WGS-84 longitude in decimal degrees
uint8 fix		# GPS fix: 0=Invalid, 1=GPS, 2=DGPS,4=RTK fixed, 5=RTK float 
uint8 sat		# Number of satellites in view
float64 hdop		# Horizontal Dilution Of Precision
float64 alt		# [m] Altitude above mean sea level
float64 geoid_height	# [m] Mean sea level height above the WGS-84 ellipsoid
