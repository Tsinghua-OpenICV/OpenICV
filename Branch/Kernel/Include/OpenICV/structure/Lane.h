#ifndef __LANE_H__
#define __LANE_H__

#include <OpenICV/structure/LanePoint.h>
#include <OpenICV/structure/LaneBoundary.h>
#include <OpenICV/structure/LaneSituation.h>
#include <msgpack.hpp>
#include <vector>

namespace icv
{
    namespace data
    {

//Basic structure of Lane
struct Lane
{
    int16 index;
    float32  speed_limit;
    float32  length;
    float32 width;
    bool bidirectional ;

    uint8 stop_state; // = 0
    uint8 STOP_STATE_UNKNOWN = 0;
    uint8 STOP_STATE_THRU = 1; // e.g. drive through at lane connection, green light
    uint8 STOP_STATE_YIELD = 2 ;// e.g. unprotected left/right turn, flashing yellow light
    uint8 STOP_STATE_STOP = 3; // e.g. red light, yellow light
    uint8 STOP_STATE_STOP_YIELD = 4; // e.g. stop sign, right turn at red light
    uint8 STOP_STATE_STOP_YIELD_ALL_WAY = 5 ;// e.g. flashing red light, all way stop sign

    // ----- Central path representation -----
    // The central_path_points field is used when central_path_type is waypoint.
    // Otherwise, central_path_coeffs should be used
    std::vector<LanePoint>  central_path_points;
    std::vector<float32> central_path_coeffs;

    uint8 central_path_type; // = 0
    uint8 CENTRAL_PATH_WAYPOINT = 0;   //discretized
    uint8 CENTRAL_PATH_LINE = 1;
    uint8 CENTRAL_PATH_CONIC = 2; // conic section, including parabola and hyperbola
    uint8 CENTRAL_PATH_POLYNOMIAL = 3 ;
    uint8 CENTRAL_PATH_BEZIER = 4;

    // ----- Boundary representation -----
    // The boundary description of current lane.
    // Not that the boundary type only describe the behaviour from current lane to neighbour lane or road shoulder
    std::vector<LaneBoundary> left_boundaries;
    std::vector<LaneBoundary> right_boundaries;

    // ----- Auxiliary information of the lane -----
    // Road situations on this line. This field could be updated with dynamic info.
    std::vector<LaneSituation> situations;

    // ---- traffic ligth position ------ 
    std::vector<float32>  traffic_light_pos;
    MSGPACK_DEFINE(index,speed_limit, length,width,bidirectional,stop_state,STOP_STATE_UNKNOWN,STOP_STATE_THRU,
    STOP_STATE_YIELD,STOP_STATE_STOP,STOP_STATE_STOP_YIELD,STOP_STATE_STOP_YIELD_ALL_WAY,central_path_type,
    CENTRAL_PATH_WAYPOINT,CENTRAL_PATH_LINE,CENTRAL_PATH_CONIC,CENTRAL_PATH_POLYNOMIAL,CENTRAL_PATH_BEZIER,
    left_boundaries,right_boundaries,situations,traffic_light_pos);
};

    }
}

#endif