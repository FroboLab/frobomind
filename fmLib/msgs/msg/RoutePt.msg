Header header
int32 cmd
float64 easting
float64 northing
float64 heading
string id
int32 nav_mode
float64 linear_vel
float64 angular_vel
float64 pause
int32 task

# cmd == 0:  Delete route plan
# cmd == 1:  Delete route plan then append route point
# cmd == 2:  Append route point
#
# mode == 0: Pure pursuit
# mode == 1: Minimize distance to AB line
#
# invalid heading, velocity, pause or task data: Use value -1000000
