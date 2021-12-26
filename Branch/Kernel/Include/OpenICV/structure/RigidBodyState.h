#ifndef __RigidBodyState_H__
#define __RigidBodyState_H__

#include<msgpack.hpp>
#include <OpenICV/structure/Pose.h>
#include <OpenICV/structure/Twist.h>
#include<OpenICV/structure/Accel.h>

#include<string>

namespace icv
{
    namespace data
    {


//Basic structure of RigidBodyState
struct RigidBodyState
{
    // ID of frame fixed to the rigid body
    string child_frame_id;

     // Location and orientatation of the object
    // geometry_msgs/PoseWithCovariance  pose;
    Pose pose;
    // float64[36] covariance;


    // Linear and angular velocity of the object
    //geometry_msgs/TwistWithCovariance twist;
    Twist twist;
    // float64[36] covariance;


    // Linear and angular acceleration of the object
    // geometry_msgs/AccelWithCovariance accel;
      Accel accel;
    // float64[36] accel;


    MSGPACK_DEFINE(child_frame_id,pose,twist,accel);
};
    }
}


#endif
