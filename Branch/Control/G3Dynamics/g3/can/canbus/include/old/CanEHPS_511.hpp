/******************************************************************************
 * @file CanEHPS_511.hpp
 * @brief CAN  receive - EHPS
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-08
 *****************************************************************************/

#ifndef __CanEHPS_511_H__
#define __CanEHPS_511_H__

#include "structureCanXP.h"

class  CanEHPS_511
{
public:

  CanEHPS_511(){};
  ~CanEHPS_511(){}; 
  void decode(){
  data_.EHPS_TranCycle = 50;
  data_.SteeringTorque = 0x7F;  
  data_.WarningLamp = 0; 
  data_.ManualInterventionSt = 0; 
  data_.DrivingMode = 0; 
  data_.AvailSt = 0; 
  data_.SteeringAngle = 0; 
  data_.SteeringWheelSpeed = 0; 

  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;

  temp =0;
  temp=(short)(pData[1]&0xff);
  data_.SteeringTorque=temp;
   
  temp =0;
  temp=(short)(pData[2]&0x03);
  data_.WarningLamp=temp;

  temp =0;
  temp=(short)(pData[2]&0x0C);
  temp=temp>>2;
  data_.ManualInterventionSt=temp;

  temp =0;
  temp=(short)(pData[3]&0x0C);
  temp=temp>>2;
  data_.DrivingMode=temp;

  temp =0;
  temp=(short)(pData[3]&0x30);
  temp=temp>>4;
  data_.AvailSt=temp;

  temp =0;
  temp1 =0;
  temp=(short)(pData[4]&0x7f);
  temp1=(short)(pData[5]&0xff);
  temp=temp<<8;
  data_.SteeringAngle=temp+temp1;

  temp =0;
  temp1 =0;
  temp=(short)(pData[6]&0x7f);
  temp1=(short)(pData[7]&0xff);
  temp=temp<<8;
  data_.SteeringWheelSpeed=temp+temp1;
  };    //end of decode()
  CanFrameEHPS_511* data() {return &data_; }; 
  int size() { return sizeof(CanFrameEHPS_511); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};

protected:

private:
  CanFrameEHPS_511 data_;
  TimestampedCanFrame d_;
};    //end of class  CanEHPS_511

#endif    //end of __CanEHPS_511_H__
