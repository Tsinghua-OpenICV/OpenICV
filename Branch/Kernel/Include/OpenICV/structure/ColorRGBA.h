#ifndef __ColorRGBA_H__
#define __ColorRGBA_H__

#include <OpenICV/structure/header.h>
#include <OpenICV/structure/Lane.h>
#include <msgpack.hpp>
#include <vector>
#include <OpenICV/structure/Point.h>



//Basic structure of ColorRGBA
struct ColorRGBA
{
  float32 r;
  float32 g;
  float32 b;
  float32 a;
  MSGPACK_DEFINE(r, g, b, a);
};


#endif
