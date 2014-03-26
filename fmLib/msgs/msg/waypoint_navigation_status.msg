Header header
# 0=idle, 1=navigate, 2=wait
int8 state
# 0=stop/standby, 1=drive, 2=turn
int8 mode
# ab line
string b_id
float64 b_easting
float64 b_northing
float64 a_easting
float64 a_northing
# robot
float64 easting
float64 northing
float64 distance_to_b
float64 bearing_to_b
float64 heading_err
float64 distance_to_ab_line
# intermediate target
float64 target_easting
float64 target_northing
float64 target_distance
float64 target_bearing
float64 target_heading_err
# controller output
float64 linear_speed
float64 angular_speed
