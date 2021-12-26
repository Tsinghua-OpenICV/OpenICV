#ifndef __LANEBOUNDARY_H__
#define __LANEBOUNDARY_H__

#include <LanePoint.h>
#include<msgpack.hpp>

//Basic structure of Lane
struct LaneBoundary
{
     LanePoint boundary_point;

     // Boundary type from between `s` and `s` of the next section start
    uint8 boundary_type;
    uint8 BOUNDARY_UNKNOWN = 0;
    uint8 BOUNDARY_DASHED_WHITE = 1; // neighbour lane has same direction.
    uint8 BOUNDARY_DASHED_YELLOW = 2; // neighbour lane has different direction.
    uint8 BOUNDARY_SOLID_WHITE = 3;  // neighbour lane has same direction, not allowed to change lane.
    uint8 BOUNDARY_SOLID_YELLOW = 4;  //neighbour lane has different direction, not allowed to change lane.
    uint8 BOUNDARY_SOLID_YELLOW_TURN = 5;  // neighbour lane has different direction, not allowed to change lane unless turning.
    uint8 BOUNDARY_CURB = 6;          // neighbour is road shoulder

    # Confidence of the lane boundary classification
    float32 confidence;
    MSGPACK_DEFINE( boundary_point,boundary_type,BOUNDARY_UNKNOWN,BOUNDARY_DASHED_WHITE,BOUNDARY_DASHED_YELLOW,BOUNDARY_SOLID_WHITE,BOUNDARY_SOLID_YELLOW,
    BOUNDARY_SOLID_YELLOW_TURN,BOUNDARY_CURB,confidence);
};

#endif

