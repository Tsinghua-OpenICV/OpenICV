//  @ Project : RADAR
//  @ File Name : sensorlrr.h
//  @ Date : 2016/12/22
//  @ Author :
//
//


#ifndef _SENSORLRR_H
#define _SENSORLRR_H

#include <cstdlib>
#include <string>
#include <sstream>
#include <cstddef>
#include <vector>
#include <msgpack.hpp>
#include "OpenICV/structure/header.h"

using namespace std ;
static const int LRRMaxTarNum = 255;

#define PI 3.1415926


typedef unsigned char uint8;
typedef float float32;
typedef double float64;
//typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;


typedef struct sVehicleInfo
{
  float64 yawrate;
  float64 velocity;
}sVehicleInfo;


typedef struct sLrrRawData
{
  uint8 measureStatus;
  int16 relSpd;
  uint16 range;
  int16 rangex;
  int16 rangey;
  int16 speedx;
  int16 speedy;
  int16 rangeRate;
  int16 rangeAccel;
  int16 angle;
  uint16 width;
  uint8 objID;
  uint16 isUpdated;
  int16 obj_amp; 
  uint16 objDynProp;
 
  int16 accx;
  int16 accy;
  int16 rangex_rms;
  int16 rangey_rms;
  int16 speedx_rms;
  int16 speedy_rms;

  int16 accx_rms;
  int16 accy_rms;
  int16 orient_rms;

  int16 objMeasState;
  uint16 objProbExist;

  int16 objClass;
  int16 ObjectOrientAngel;
  uint16 ObjectLength;
  uint16 ObjectWidth;
  
}sLrrRawData;

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
Header header_ ;
int targetnum_;
sLrrProcData alldata_[LRRMaxTarNum];
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



typedef struct sLrrConfigdata201
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
}stLrrCfgdata201; 

typedef struct sLrrConfigdata202
{
  uint16 filterCfg_valid;
  uint16 filterCfg_active;
  uint16 filterCfg_type;
  uint16 filterCfg_index;
  uint16 filterCfg_min_NoOfob;
  
 uint16 filterCfg_min_distance;
 uint16 filterCfg_max_distance;
}stLrrCfgdata202; 

typedef struct sLrrCfgdata203
{
  uint16 NofClusterFilterCfg;
  uint16 NofobjectFilterCfg;

}stLrrCfgdata203; 

typedef struct sLrrCfgdata204
{
  uint16 NofClusterFilterCfg;
  uint16 NofobjectFilterCfg;

}stLrrCfgdata204; 









#endif  //_SENSORlrr_H
