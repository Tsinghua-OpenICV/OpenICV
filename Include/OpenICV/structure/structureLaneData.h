#ifndef _laneresult_H
#define _laneresult_H


#include <cstddef>
#include <cstdlib>
#include <string>
#include <sstream>
#include <vector>
#include <msgpack.hpp>

#include "OpenICV/structure/header.h"


typedef unsigned char uint8;
typedef float float32;
typedef double float64;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;

struct ld_Point
{
    float32 x;
    float32 y;
    MSGPACK_DEFINE(x,y);
};

struct ld_LaneParam
{
    vector<ld_Point> lane_Single;
    MSGPACK_DEFINE(lane_Single);
};

struct ld_Coeff
{
    uint8 id;
    float32 a;
    float32 b;
    float32 c;
    float32 d;
    float32 a_img;
    float32 b_img;
    float32 c_img;
    float32 d_img;
    MSGPACK_DEFINE(id,a,b,c,d,a_img,b_img,c_img,d_img);
};

struct ld_Frame
{
    Header header ;
    vector<ld_Coeff> lane_Coeff ;
    vector<ld_LaneParam> lane_Pixel ;
    vector<ld_LaneParam> lane_World ;
    float32 bias_dis;
    float32 bias_theta;
    float32 lane_width;
    float32 curve_radius;
    int8 cl_flag;
    MSGPACK_DEFINE(header, lane_Coeff, lane_Pixel, lane_World, bias_dis, bias_theta,lane_width, curve_radius, cl_flag);
};


#endif 
