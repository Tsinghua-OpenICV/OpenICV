#ifndef Point_h 
#define Point_h 

#include <msgpack.hpp>

namespace icv
{
    namespace data
    {


struct Point
{
  
   float x;
   float y;
   float z;

    MSGPACK_DEFINE(x,y,z) ;
};
    }
}
#endif
			
