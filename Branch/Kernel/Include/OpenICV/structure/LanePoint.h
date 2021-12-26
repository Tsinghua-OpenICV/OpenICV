#ifndef __LANEPOINT_H__
#define __LANEPOINT_H__

#include <OpenICV/structure/Point.h>
#include <msgpack.hpp>

typedef float float32;

namespace icv
{
    namespace data
    {

//Basic structure of LanePoint
struct LanePoint
{
    Point position;
    
    // Distance from lane start to current point. The distance of the first point should be zero
    float32 s;

    // Slope at current position of the road
    // Can be used to control the throttle
    float32 slope;

    // Road curvature at current position of the road
    // Can be used to slow down before turning
    float32 curvature;

    // The yaw angle of tangent line (in radian)
    float32 tangent;

    // Road width at current position
    // Can be used to determine the carefulness of driving
    float32 width;
    MSGPACK_DEFINE(s, slope,curvature, tangent,width);

};
    }
}


#endif
