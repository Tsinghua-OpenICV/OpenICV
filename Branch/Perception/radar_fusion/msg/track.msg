Header header
uint8 id
uint8 flag_repeat
uint8 object_type
uint8 tracks_lane  #get the current lane which the tracks are in.
geometry_msgs/PoseWithCovariance object_box_center
geometry_msgs/Point object_box_size
geometry_msgs/Point[] contour_points
geometry_msgs/Point obs_position 
geometry_msgs/Point velocity                 
geometry_msgs/Point acceleration
geometry_msgs/Point Box3D_size # x=length y=width z=height
geometry_msgs/PoseWithCovariance Box3D_center #camera
float32 obj_ori # _object_orientation yaw