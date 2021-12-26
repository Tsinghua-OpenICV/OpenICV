#ifndef __CANFRAME_H__
#define __CANFRAME_H__

#include <OpenICV/structure/header.h>
#include <OpenICV/structure/Lane.h>
#include <msgpack.hpp>
#include <vector>
#include <OpenICV/structure/Point.h>

namespace icv
{
    namespace data
    {

//Basic structure of Map
struct Map
{
    Header header;
    // Whether the map is in a structured environment
    bool in_junction; // = True

    // Target lane index at the end of the section.
    std::vector<int8>  exit_lane_index;

    // Lanes if it's in a structured road, should be sorted by ascending index
    // The index is starting from right most lane, i.e. the right most lane is indexed as 0
    std::vector<Lane>  lanes;

    // Road area if in junction
    //geometry_msgs/Polygon drivable_area;
    std:: vector<Point> drivable_area;

    // # Next unit
  //  geometry_msgs/Polygon next_drivable_area;
    std:: vector<Point>  next_drivable_area;
  
     std::vector<Lane> next_lanes;
    int8 next_road_id;
    MSGPACK_DEFINE(in_junction,exit_lane_index,lanes,next_lanes,drivable_area,next_drivable_area);

};
    }
}


#endif
