

#ifndef _SENSORSRR_H
#define _SENSORSRR_H

#include <cstdlib>
#include <string>
#include <sstream>
#include <msgpack.hpp>
#include "OpenICV/structure/header.h"

using namespace std;
static const int SRRMaxTarNum = 255;


typedef unsigned char uint8;
typedef float float32;
typedef double float64;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;

typedef struct sSrrRawData
{
  uint8 objID;
  uint16 isUpdated;
  int16 rangex;
  int16 rangey;
  int16 speedx;
  int16 speedy;
  uint16 width;
  int16 obj_amp; 
  uint16 trackIndex;
  uint16 trackIndex2;
  uint16 trackLifeTime;
}sSrrRawData;

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
  Header header_ ;
  int targetnum_;
  sSrrProcData alldata_[SRRMaxTarNum];
  MSGPACK_DEFINE(header_,targetnum_,alldata_);
};

typedef struct _CANMsg
{
  uint8 DLC      :4;//Data length,0<=DLC<=8
  uint8 reserved :2;
  uint8 RTR      :1;//1 for remote request
  uint8 FF       :1;//Frame form. 1 = Extended, 0 = Standard
  uint32 ID;//ID of message
  uint8 data[8];//Data of message
}stCANMsg;



typedef struct sSrrConfigdata201
{
  uint16 nvmReadStatus;
  uint16 nvmWriteStatus;
  uint16 maxDistanceCfg;
  uint16 sensorID;
  uint16 sortIndex;
  uint16 outputCfg;
  uint16 rcs_thred;
  uint16 powerCfg;
  uint16 motionRxstate;
  uint16 sendExtInfoCfg;
  uint16 sendQualityCfg;
  uint16 ctrlRelayError;
  uint16 voltageError;
  uint16 temporaryError;
  uint16 temperatureError;
  uint16 interface;
  uint16 persistentError;
}stSrrCfgdata201; 

typedef struct sSrrConfigdata202
{
  uint16 filterCfg_valid;
  uint16 filterCfg_active;
  uint16 filterCfg_type;
  uint16 filterCfg_index;
  uint16 filterCfg_min_NoOfob;
  
 uint16 filterCfg_min_distance;
 uint16 filterCfg_max_distance;
}stSrrCfgdata202; 

typedef struct sSrrCfgdata203
{
  uint16 NofClusterFilterCfg;
  uint16 NofobjectFilterCfg;

}stSrrCfgdata203; 

typedef struct sSrrCfgdata204
{
  uint16 NofClusterFilterCfg;
  uint16 NofobjectFilterCfg;

}stSrrCfgdata204; 






#endif  //_SENSORsrr_H
