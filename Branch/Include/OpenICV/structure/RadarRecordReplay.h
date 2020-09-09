#ifndef _RecordReplayRadar_H
#define _RecordReplayRadar_H

#include <cstdlib>
#include <string>
#include <sstream>
#include <msgpack.hpp>
#include "OpenICV/structure/header.h"

using namespace std;
static const int SRRMaxTarNum = 255;
static const int LRRMaxTarNum = 255;

typedef unsigned char uint8;
typedef float float32;
typedef double float64;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;

typedef struct sSrrProcData {
  uint8 id;
  float32 x;
  float32 y;
  float32 relspeedx;
  float32 relspeedy;
  float32 obj_amp; 
  uint16 trackIndex;
  uint16 trackIndex2;
  float32 trackLifeTime;
  uint8 flag;
  MSGPACK_DEFINE(id, x, y, relspeedx, relspeedy,  obj_amp, trackIndex,trackIndex2,trackLifeTime,flag);
}sSrrProcData;

struct RadarSideOut {
public:
  RadarSideOut():targetnum_(0) {};
  Header header_ ;
  int targetnum_;
  sSrrProcData alldata_[SRRMaxTarNum];
  MSGPACK_DEFINE(header_, targetnum_,alldata_);
};


typedef struct sLrrProcData {
  uint8 id;
  float32 range;
  float32 rangerate;
  float32 angle;
  float32 x;
  float32 y;
  float32 relspeedx;
  float32 relspeedy;
  float32 obj_amp; 
  uint16 objDynProp; 
  uint8 flag;

  float32 accx;
  float32 accy;

  float32 rangex_rms;
  float32 rangey_rms;
  float32 speedx_rms;
  float32 speedy_rms;

  float32 accx_rms;
  float32 accy_rms;

  float32 orient_rms;

  int16 objMeasState;
  uint16 objProbExist;

  int16 objClass;
  float32 ObjectOrientAngel;
  float32 ObjectLength;
  float32 ObjectWidth;
  MSGPACK_DEFINE(   id,range, rangerate, angle, x, y, relspeedx, relspeedy,
   obj_amp, objDynProp, flag,  accx, accy,  rangex_rms, rangey_rms,
   speedx_rms, speedy_rms, accx_rms, accy_rms, orient_rms, objMeasState,
   objProbExist, objClass,ObjectOrientAngel,ObjectLength,ObjectWidth);

}sLrrProcData;

struct RadarLongOut {
RadarLongOut():targetnum_(0) {};
Header header_ ;
int targetnum_;
//vector<sLrrProcData>alldata_;
sLrrProcData alldata_[LRRMaxTarNum];
MSGPACK_DEFINE(header_, targetnum_,alldata_)
};







#endif  
