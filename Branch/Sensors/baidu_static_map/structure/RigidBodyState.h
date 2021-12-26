#ifndef __RigidBodyState_H__
#define __RigidBodyState_H__

#include<msgpack.hpp>
#include<string>

//Basic structure of RigidBodyState
struct Map
{
    // ID of frame fixed to the rigid body
    string child_frame_id;

     // Location and orientatation of the object
    // geometry_msgs/PoseWithCovariance  pose;
    float64[36] pose;


    // Linear and angular velocity of the object
    //geometry_msgs/TwistWithCovariance twist;
    float64[36] twist;
    // Linear and angular acceleration of the object
    // geometry_msgs/AccelWithCovariance accel;
    float64[36] accel;


    MSGPACK_DEFINE(child_frame_id,pose,twist,accel);
};


#endif
