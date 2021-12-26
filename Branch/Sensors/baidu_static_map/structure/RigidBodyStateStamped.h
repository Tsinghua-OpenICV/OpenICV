#ifndef __RIGIDBODYSTATESTAMPED_H__
#define __RIGIDBODYSTATESTAMPED_H__

#include <Header.h>
#include <RigidBodyState.h>
#include<msgpack.hpp>


using namespace std;

//Basic structure of RigidBodyStateStamped
struct RigidBodyStateStamped
{
  // Stamped version of RigidBodyState
    Header header;
    RigidBodyState state;
    MSGPACK_DEFINE(header,state);
};

#endif
