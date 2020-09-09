# Header header
std_msgs/Header header

#ObstacleId
int32 obstacle_id

# Obstacle Position to the world frame
# unit=m
geometry_msgs/Point position
float32 theta

# Obstacle Relative Linear velocity refer to the world frame
# (World linear velocity in world frame)
# unit=m/s
geometry_msgs/Vector3 velocity

# Obstacle BondingBox length, along obstacle's (+y) direction defined in ObsPos
# unit=m
float32 length
# Obect BondingBox width,  along obstacle's (+x) direction defined in ObsPos
# unit=m
float32 width
# Obect BondingBox height, (+z)
# unit=m
float32 height

# corner points
geometry_msgs/Point[] polygon_points

# time elapsed after Obstacle detection
# unit=s
float32 life

#Obect Classfication 
#TODO detail of enums     0 unclassified;1 unknown small; 2 unknown big;3 pedestrian;4 bike;5 car; 6 truck; 
 
int16 OBSTACLE_TYPE_UNCLASSIFIED = 0            # 0: No classification determined (yet)
int16 OBSTACLE_TYPE_UNKNOWN_SMALL = 1           # 1: Relatively small structure like pole, tree, single bush, etc. which does not fit to other classes.
int16 OBSTACLE_TYPE_UNKNOWN_BIG = 2             # 2: Bigger structure which does not fit other classes.
int16 OBSTACLE_TYPE_PEDESTRIAN = 3              # 3: Pedestrian, usually determined by moving behaviour.
int16 OBSTACLE_TYPE_BIKE = 4                    # 4: bike, motor bike
int16 OBSTACLE_TYPE_CAR = 5                     # 5: Passenger car.
int16 OBSTACLE_TYPE_TRUCK = 6                   # 6: Big vehicle, bus, truck.
int16 OBSTACLE_TYPE_FENCE = 7                   # 7: continuous fence on the side of the read
int16 OBSTACLE_TYPE_PLANTS = 8                  # 8: continuous bushs, flowers, etc.

int16 classification
