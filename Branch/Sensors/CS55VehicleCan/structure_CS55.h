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
    
    short RealStrAngle;// (-4800, 4800) after *0.1 is steering angle in degree 
    uint8_t RealShiftPosition; //0 ->P;1-> R; 2->N;3->D
    uint8_t CurrentDrvMode; //0-> auto drive;1 HUMAN drive; 2 AEB
    uint8_t SysSwStatus; //bit0: steerCtrl, bit1:speedCtrl, bit2:leftlight, 
    //bit3:right light, bit 4:clean state bit 5:brush state;bit 6:ash bin pos; bit 7: ash bin state ----> 0 switch off, 1 switch on; 
    uint8_t VehicleSysFault;//bit 1:brake, bit 2:steer, bit 3:accelerate----> 0 normal, 1 error;
    short VehicleSpeed;// after *0.1 is vehicle speed in km/h 
    
    MSGPACK_DEFINE(RealStrAngle,RealShiftPosition,CurrentDrvMode,SysSwStatus,VehicleSysFault, VehicleSpeed);

};

struct CS55_CONTR_MCU
{
    short TargetStrAngle;// (-4800,4800) after *0.1 is steering angle in degree 
    short  TargetAccelReq;// (0,1000)
    short  TargetDecelReq;  //(0,100)---->(0-5m/s2)
    uint8_t TargetDrivingStatus; // 0 decel; 1 acceler;2 braking 
    uint8_t   TargetModeSelect;// 0 human driving ;1 automated driving
    uint8_t   TargetShiftPosition; // 0 ->P;1-> R; 2->N;3->D  
    uint8_t   BodyEleControl; //bit4: Turn left; bit 5 : trun right; bit 6: horn ：0 off <->1 on
    uint8_t   SpecificControl;
 
    MSGPACK_DEFINE(TargetStrAngle,TargetAccelReq,TargetDecelReq, TargetDrivingStatus,TargetModeSelect,TargetShiftPosition,BodyEleControl,SpecificControl);

};

typedef data::icvStructureData<CS55_READ_MCU>    icvCS55_READ_MCU;
typedef data::icvStructureData<CS55_CONTR_MCU>    icvCS55_CONTR_MCU;

}


#endif // __STRUCTURECANCS55_H__
