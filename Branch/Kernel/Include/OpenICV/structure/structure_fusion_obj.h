// #ifndef UDP_PROTOCOL_H
// #define UDP_PROTOCOL_H

#include <msgpack.hpp>

#define EthBufferMaxSize 5888u

typedef unsigned char  uint8;
typedef unsigned short uint16;
typedef float     float32;

struct OBJ_FUSION
{
public:

uint8 Track_ID;
uint8 Track_Object_Class;

float32 Track_Class_Probability;
float32 Track_Lat_Distance;
float32 Track_Long_Distance;
float32 Track_Relative_Lat_Velocity;
float32 Track_Relative_Long_Velocity;
float32 Track_Abs_Lat_Velocity;
float32 Track_Abs_Long_Velocity;
float32 Track_Height;
float32 Track_Height_STD;
float32 Track_Length;
float32 Track_Length_STD;
float32 Track_Width;
float32 Track_Width_STD;
float32 Track_Existence_Probability;

uint16 Track_Age;
uint8 Track_time_since_update;
uint8 Track_Motion_Orientation;
uint8 Track_Measuring_Status;
uint8 Track_Motion_Category;
bool match_state;

MSGPACK_DEFINE(Track_ID,Track_Object_Class,Track_Class_Probability,Track_Lat_Distance,Track_Long_Distance,Track_Relative_Lat_Velocity,Track_Relative_Long_Velocity,
Track_Abs_Lat_Velocity,Track_Abs_Long_Velocity,Track_Height,Track_Height_STD,Track_Length,Track_Length_STD,Track_Width
,Track_Width_STD,Track_Existence_Probability,Track_Age,Track_Motion_Orientation,Track_Measuring_Status,Track_Motion_Category,match_state);
};

struct FUSION_ARRAY
{
    OBJ_FUSION objs_fusion[32];
    MSGPACK_DEFINE(objs_fusion);
    /* data */
};

// #endif