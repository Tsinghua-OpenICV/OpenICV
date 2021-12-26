#ifndef __MARKERARRAY_H__
#define __MARKERARRAY_H__

#include <OpenICV/structure/Marker.h>
#include <msgpack.hpp>
#include <vector>
namespace icv{  
   namespace data {


//Basic structure of MarkerArray
struct MarkerArray
{
   //Marker[] markers
   std::vector<Marker> markers;


   MSGPACK_DEFINE( markers);
};
   }
}

#endif 
