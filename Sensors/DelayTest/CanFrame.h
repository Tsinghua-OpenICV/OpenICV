#ifndef __CANFRAME_H__
#define __CANFRAME_H__

#include <cstddef>
#include <cstddef>
#include <cstdlib>
#include <string>
#include <sstream>


typedef signed char             int8_t;   
typedef short int               int16_t;  
typedef int                     int32_t;  
typedef long int                int64_t;  
 
typedef unsigned char           uint8_t;  
typedef unsigned short int      uint16_t;  
typedef unsigned int            uint32_t;  
typedef unsigned long int       uint64_t;  


typedef uint64_t road_time_t;
typedef int32_t road_timerange_t;

/// Basic structure of a CAN frame
struct CanFrame
{
    static const std::size_t MAX_CAN_MSG_LENGTH = 8;

    uint16_t id;   //uint32_t  change to uint16_t for standard ID
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
