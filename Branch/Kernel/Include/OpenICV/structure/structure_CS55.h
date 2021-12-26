//  created:    2020/10/15
//
//  author:     Kun JIANG
//              Copyright Tsinghua University
// 
//  version:    $Id: $
//
//  purpose:    

#ifndef __STRUCTURECANCS55_H__
#define __STRUCTURECANCS55_H__

#include "OpenICV/Basis/icvStructureData.hxx"
#include <stdint.h>
#include <msgpack.hpp>
#include <vector> 
#include <string>
namespace icv
{


struct CS55_READ_MCU
{
    
    double RealStrAngle;
    uint8_t RealShiftPosition,CurrentDrvMode,SysSwStatus,VehicleSysFault;
    double VehicleSpeed;
    
    MSGPACK_DEFINE(RealStrAngle,RealShiftPosition,CurrentDrvMode,SysSwStatus,VehicleSysFault, VehicleSpeed);

};

struct CS55_CONTR_MCU
{
    short TargetStrAngle,TargetAccelReq,TargetDecelReq;  
    uint8_t TargetDrivingStatus,TargetModeSelect,TargetShiftPosition,BodyEleControl,SpecificControl;
 
    MSGPACK_DEFINE(TargetStrAngle,TargetAccelReq,TargetDecelReq, TargetDrivingStatus,TargetModeSelect,TargetShiftPosition,BodyEleControl,SpecificControl);

};

typedef data::icvStructureData<CS55_READ_MCU>    icvCS55_READ_MCU;
typedef data::icvStructureData<CS55_CONTR_MCU>    icvCS55_CONTR_MCU;

}


#endif // __STRUCTURECANCS55_H__
