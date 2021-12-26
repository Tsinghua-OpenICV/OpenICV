#ifndef __RIGIDBODYSTATESTAMPED_H__
#define __RIGIDBODYSTATESTAMPED_H__

#include <OpenICV/structure/Header.h>
#include <OpenICV/structure/RigidBodyState.h>
#include<msgpack.hpp>


namespace icv
{
    namespace data
    {
//Basic structure of RigidBodyStateStamped
struct RigidBodyStateStamped
{
  // Stamped version of RigidBodyState
    Header header;
    RigidBodyState state;
    MSGPACK_DEFINE(header,state);
};
    }
}
    

#endif
