/*********************************************************************
//  created:    2017/12/15
//
//  author:     Kun JIANG
//              Copyright Tsinghua University
// 
//  version:    $Id: $
//
//  purpose:   
*********************************************************************/

#ifndef __CanTUAN121_h__
#define __CanTUAN121_h__


#include "OpenICV/structure/structureCanTuan.h"


class  CanTuan121
{
public:

  CanTuan121(){};
  ~CanTuan121(){}; 
  void decode(){


  data_.aPedalPercent=0;
  data_.aPedalPercentSt=0; 
  data_.ThrottleGradientSize=0;
  data_.ThrottleGradientPN=0;
  data_.EngNeutralTorque=0; 
  uint8 *pData = d_.frame.data;
  int16 temp,temp1;
 
  temp =0;
  temp1=0;
  temp=(short)(pData[1]&0xF0);
  temp=temp>>4;
  temp1=(short)(pData[2]&0x0F);
  temp1=temp1<<4;
  temp1+=temp;
  data_.aPedalPercent=temp1*0.4;

  temp =0;
  temp1=0;
  temp=(short)(pData[2]&0xE0);
  temp=temp>>5;
  temp1=(short)(pData[3]&0x1F);
  temp1=temp1<<3;
  temp1+=temp;
  data_.ThrottleGradientSize=temp1*25;

  temp =0;
  temp=(short)(pData[2]&0x10);
  temp=temp>>4;
  data_.aPedalPercentSt=temp;

  temp =0;
  temp=(short)(pData[3]&0x20);
  temp=temp>>5;
  data_.ThrottleGradientPN=temp;

  temp =0;
  temp=(short)(pData[4]&0x20);
  temp=temp>>5;
  data_.EngNeutralTorque=temp;
  };
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameTouranMotor_20); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
  CanFrameTouranMotor_20 data_;
TimestampedCanFrame d_;
};



#endif

