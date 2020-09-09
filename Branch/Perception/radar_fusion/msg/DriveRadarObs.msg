# Header header
std_msgs/Header header

# Obstacle Id
int32 obstacle_id

# Obstacle range to the radar 
# unit = m
float64 range

# Obstacle azimuth angle to the radar perpendicular bisectors 
# (+) clockwise
# unit = degree
float64 theta

# Obstacle radial velocity relative to the radar 
# (+) away from the radar
# unit = m/s
float64 radial_velocity

# Obstacle tangential velocity relative to the radar 
# (+) clockwise
# unit = m/s
float64 tangential_velocity

# Obstacle radial acceleration relative to the radar 
# (+) away from the radar
# unit = m/s
float64 radial_acceleration

# Obstacle Radar Cross-Section
# unit = dBsm
int32 rcs  

# Obstacle width
# unit = m
float64 width 
