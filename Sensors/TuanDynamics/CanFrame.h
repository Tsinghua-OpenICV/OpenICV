#ifndef __CANFRAME_H__
#define __CANFRAME_H__

#include <cstddef>
#include <cstddef>
#include <cstdlib>
#include <string>
#include <sstream>
typedef uint64_t road_time_t;
typedef int32_t road_timerange_t;

/// Basic structure of a CAN frame
struct CanFrame
{
    static const std::size_t MAX_CAN_MSG_LENGTH = 8;

    uint32_t id;
    uint8_t dlc;
    uint8_t data [MAX_CAN_MSG_LENGTH];
};

/// CAN structure with timestamping
struct TimestampedCanFrame
{
    CanFrame frame;
    road_time_t time;
    road_timerange_t timerange;
};

#endif
