

#ifndef _icvFrame_H
#define _icvFrame_H
#include <stdint.h>
#include <msgpack.hpp>
#include <vector> 
#include <string>
#include "OpenICV/Basis/icvStructureData.hxx"

using namespace std;
namespace icv{
 struct buffFrame {
  std::int64_t time_stamp_; 
  unsigned long length_;
  string data;
  MSGPACK_DEFINE(time_stamp_,length_,data);
}; 
typedef data::icvStructureData<buffFrame>    icvbuffFrame;
}




#endif // _icvFrame_H


