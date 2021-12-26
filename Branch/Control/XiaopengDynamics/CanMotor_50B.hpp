//
//  created:    2017/12/15
//
//  author:     Kun JIANG
//              Copyright Tsinghua University
// 
//  version:    $Id: $
//
//  purpose:    
//

#ifndef __CANMOTOR_50B_H__
#define __CANMOTOR_50B_H__

#include "structureCanXP.h"


class  CanMotor_50B
{
public:

  CanMotor_50B(){};
  ~CanMotor_50B(){}; 
  void decode(){

  data_.Motor_TranCycle=20;
  data_.AccPedal_CCP=0; 
  data_.BrkPedal_CCP=0; 
  data_.EpbState_CCP=0; 
  data_.GearState_CCP=0; 

   uint8 *pData = d_.frame.data;
   int16 temp,temp1,temp2,temp3;
   
   temp =0;
   temp=(short)(pData[3]&0xff);
   data_.EpbState_CCP=temp;
   
   temp1=0;
   temp1=(short)(pData[4]&0xff);
   data_.GearState_CCP=temp1;

   temp2=0;
   temp2=(short)(pData[6]&0xff);
   data_.BrkPedal_CCP=temp2;   

   temp3=0;
   temp3=(short)(pData[7]&0xff);
   data_.AccPedal_CCP=temp3; 
  };
  
  CanFrameMotor_50B * data() {return &data_; }; 
  int size() { return sizeof(CanFrameMotor_50B); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};

protected:

private:
  CanFrameMotor_50B data_;
  TimestampedCanFrame d_;
};


#endif



