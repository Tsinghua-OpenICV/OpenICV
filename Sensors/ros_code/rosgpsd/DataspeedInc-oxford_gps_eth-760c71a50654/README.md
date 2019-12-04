Ethernet interface to OxTS GPS receivers using the NCOM packet structure
===============================

![](RT3v2.jpg)

Tested with the RT3000v2 receiver.

### Published Topics
- `gps/fix` ([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html)) GPS position in geodetic coordinates (latitude and longitude)
- `gps/vel` ([geometry_msgs/TwistWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html)) GPS velocity in ENU frame
    - `linear.x` Velocity due East in m/s
    - `linear.y` Velocity due North in m/s
    - `linear.z` Velocity up in m/s
- `gps/odom` ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)) GPS position and heading in UTM frame, and velocity in vehicle body frame
    - `pose.position.x` UTM Easting in meters
    - `pose.position.y` UTM Northing in meters
    - `pose.orientation` Yaw quaternion representing the heading in UTM. Keep in mind that this UTM heading is relative to the UTM grid, which is not necessarily aligned with due East. This phenomenon is known as "grid convergence". See [Wikipedia](https://en.wikipedia.org/wiki/Transverse_Mercator_projection#Coordinates,_grids,_eastings_and_northings) for details.
    - `twist.linear.x` Local frame forward velocity of the vehicle in m/s
    - `twist.linear.y` Local frame lateral velocity of the vehicle in m/s
    - `twist.angular.z` Yaw rate of the vehicle in rad/s
- `imu/data` ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)) Orientation, angular rates, and linear accelerations reported directly from the OxTS unit
- `gps/pos_type` ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html)) Position solution type reported by the OxTS unit (point position, differential pseudorange, RTK float, RTK integer, etc.)
- `gps/nav_status` ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html)) Navigation status reported by the OxTS unit. `fix`, `vel`, `odom`, and `imu/data` topics will not be published until this topic contains the string "READY".
- `gps/time_ref` ([sensor_msgs/TimeReference](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/TimeReference.html)) UTC equivalent of the GPS timestamp reported by the OxTS unit.
### Parameters
- `interface` Restrict to single network interface, example: `eth0`. Default `<empty>`
- `ip_address` Local IP listen address. Default `<empty>`
- `port` Local UDP listen port. Default `3000`
- `frame_id_gps` The frame-ID for geodetic GPS position and IMU. Default `gps`
- `frame_id_vel` The frame-ID for ENU GPS velocity. Default `enu`
- `frame_id_odom` The frame-ID for UTM odometry. Default `utm`

### Example usage
```
rosrun oxford_gps_eth gps_node
```
OR
```
roslaunch oxford_gps_eth gps.launch
```

### FAQ
I see ```Connected to Oxford GPS at <ip_address>:3000```, but GPS position and velocity messages are not being published.

Data is not published until the OxTS unit reports itself as being completely initialized.  Drive around faster than 5 m/s for a short period of time after powering on the OxTS unit to initialize the sensor.  The `gps/nav_status` string topic indicates the status of the initialization process. Once this string shows "READY", the GPS position and velocity messages should start publishing.

