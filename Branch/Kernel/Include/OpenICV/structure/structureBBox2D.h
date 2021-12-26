#ifndef __STRUCTURE_BBOX_2D_H__
#define __STRUCTURE_BBOX_2D_H__

#include <cstddef>
#include <cstdlib>
#include <string>
#include <sstream>
#include <vector>
#include "OpenICV/structure/header.h"

struct BoundingBox2d
{
double probability;
float xmin;
float ymin;
float xmax;
float ymax;
int label ;
MSGPACK_DEFINE(probability,xmin, ymin, xmax, ymax);

};

struct BoundingBoxes2d
{
  Header header;
  vector<BoundingBox2d> bounding_boxes;
  MSGPACK_DEFINE(header, bounding_boxes);
};

#endif