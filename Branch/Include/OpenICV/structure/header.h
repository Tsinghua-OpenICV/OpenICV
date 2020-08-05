#ifndef __HEADER_H__
#define __HEADER_H__

#include <cstdlib>
#include <string>
#include <sstream>
#include <msgpack.hpp>
#include <time.h>

using namespace std;

struct Header
{
  // Header()
  //   : seq(0)
  //   , stamp()
  //   , frame_id()  {
  //   }
    uint32_t  seq;
    time_t stamp;
    // char frame_id[16];
    // string frame_id;
    MSGPACK_DEFINE(seq, stamp) ;
};

#endif