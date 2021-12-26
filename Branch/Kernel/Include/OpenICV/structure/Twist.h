#ifndef __TWIST_H__
#define __TWIST_H__

#include<msgpack.hpp>
#include <OpenICV/structure/Vector3.h>
#include<string>

namespace icv
{
    namespace data
    {

//Basic structure of TWIST
struct Twist
{
    // ID of frame fixed to the Twist
    Vector3 linear;
    Vector3 angular;

    MSGPACK_DEFINE(linear,angular);
};
    }
}


#endif
