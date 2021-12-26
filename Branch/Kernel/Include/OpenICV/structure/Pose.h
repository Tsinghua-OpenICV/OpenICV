#ifndef __POSE_H__
#define __POSE_H__

#include <OpenICV/structure/Point.h>
#include <OpenICV/structure/Quaternion.h>
#include <msgpack.hpp>

namespace icv
{
    namespace data
    {


//Basic structure of Pose
struct Pose
{
    Point position;
   Quaternion orientation;
   

    MSGPACK_DEFINE(position, orientation);
};
    }
}



#endif 
