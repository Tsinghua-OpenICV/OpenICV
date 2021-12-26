/******************************************************************************
 * @file CanVST_210.hpp
 * @brief CAN  receive - VST
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-08
 *****************************************************************************/

#ifndef __CanVST_210_H__
#define __CanVST_210_H__

#include "structureCanXP.h"


class  CanVST_210
{
public:

  CanVST_210(){};
  ~CanVST_210(){}; 
  void decode(){
  data_.VST_TranCycle = 10;
  data_.EstopSignal = 0; 
  data_.AutoDriveEnable = 0; 
  data_.ParkingBrakeSwitch = 0; 
  data_.EPBWarningLamp = 0; 
  data_.EPBDriveMode = 0; 
  data_.BrakePedalPosition = 0; 
  data_.EBSRedWarningSignal = 0; 
  data_.LongitudinalAcceleration = 0; 

  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;
  
  temp =0;
  temp=(short)(pData[0]&0x01);
  data_.EstopSignal=temp;
   
  temp =0;
  temp=(short)(pData[0]&0x02);
  temp=temp>>1;
  data_.AutoDriveEnable=temp;

  temp =0;
  temp=(short)(pData[0]&0x0C);
  temp=temp>>2;
  data_.ParkingBrakeSwitch=temp; 

  temp =0;
  temp=(short)(pData[0]&0x30);
  temp=temp>>4;
  data_.EPBWarningLamp=temp; 

  temp =0;
  temp=(short)(pData[0]&0x40);
  temp=temp>>6;
  data_.EPBDriveMode=temp; 

  temp =0;
  temp=(short)(pData[1]&0xff);
  data_.BrakePedalPosition=temp;

  temp =0;
  temp=(short)(pData[5]&0x0C);
  temp=temp>>2;
  data_.EBSRedWarningSignal=temp; 

  temp =0;
  temp=(short)(pData[7]&0xff);
  data_.LongitudinalAcceleration=temp;
  };    //end of decode()
  CanFrameVST_210* data() {return &data_; }; 
  int size() { return sizeof(CanFrameVST_210); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};

protected:

private:
  CanFrameVST_210 data_;
  TimestampedCanFrame d_;
};    //end of class  CanVST_210

#endif    //end of __CanVST_210_H__
