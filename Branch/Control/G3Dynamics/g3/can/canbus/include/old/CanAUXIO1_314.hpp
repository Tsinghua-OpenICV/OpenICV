/******************************************************************************
 * @file CanAUXIO1_314.hpp
 * @brief CAN  receive - AUXIO1
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-08
 *****************************************************************************/

#ifndef __CanAUXIO1_314_H__
#define __CanAUXIO1_314_H__

#include "structureCanXP.h"


class  CanAUXIO1_314
{
public:

  CanAUXIO1_314(){};
  ~CanAUXIO1_314(){}; 
  void decode(){
  data_.AUXIO1_TranCycle = 20;
  data_.LowBeamSt = 0; 
  data_.BrakeLightSt = 0; 
  data_.TurnRightSt = 0; 
  data_.TurnLeftSt = 0; 
  data_.HighBeamSt = 0; 
  data_.HazardEnvironmentLight = 0; 

  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;
  
  temp =0;
  temp=(short)(pData[0]&0x03);
  data_.LowBeamSt=temp;

  temp =0;
  temp=(short)(pData[1]&0x03);
  data_.BrakeLightSt=temp;

  temp =0;
  temp=(short)(pData[1]&0x0C);
  temp=temp>>2;
  data_.TurnRightSt=temp;

  temp =0;
  temp=(short)(pData[1]&0x30);
  temp=temp>>4;
  data_.TurnLeftSt=temp;

  temp =0;
  temp=(short)(pData[2]&0xC0);
  temp=temp>>6;
  data_.HighBeamSt=temp;

  temp =0;
  temp=(short)(pData[4]&0x0C);
  temp=temp>>2;
  data_.HazardEnvironmentLight=temp;
  };    //end of decode()
  CanFrameAUXIO1_314* data() {return &data_; }; 
  int size() { return sizeof(CanFrameAUXIO1_314); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};

protected:

private:
  CanFrameAUXIO1_314 data_;
  TimestampedCanFrame d_;
};    //end of class  CanAUXIO1_314

#endif    //end of __CanAUXIO1_314_H__
