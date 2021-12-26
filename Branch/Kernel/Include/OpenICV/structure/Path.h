#ifndef __Path_H__
#define __PATH_H__

#include <OpenICV/structure/Header.h>
#include <OpenICV/structure/Pose.h>
#include <msgpack.hpp>

namespace icv
{
    namespace data
    {


struct Path
{
  
   Header  header;
    Pose pose;
    MSGPACK_DEFINE(header, pose) ;
};
    }
}
#endif