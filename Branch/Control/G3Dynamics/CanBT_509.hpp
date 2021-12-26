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

#ifndef __CANBT_509_H__
#define __CANBT_509_H__

#include "structureCanXP.h"


class  CanBT_509
{
public:

  CanBT_509(){};
  ~CanBT_509(){}; 
  void decode(){

  data_.BT_TranCycle=20;
  data_.State_Braking_CCP=0; 
  data_.State_TurningLight_CCP=0; 
  data_.CurDriveMode_CCP=0; 


   uint8 *pData = d_.frame.data;
   int16 temp,temp1,temp2,temp3;
   
   temp =0;
   temp=(short)(pData[0]&0xff);
   data_.State_TurningLight_CCP=temp;
   
   temp1=0;
   temp1=(short)(pData[1]&0xff);
   data_.CurDriveMode_CCP=temp1;

   temp2=0;
   temp2=(short)(pData[2]&0xff);
   data_.State_Braking_CCP=temp2;   
  };
  CanFrameBT_509 * data() {return &data_; }; 
  int size() { return sizeof(CanFrameBT_509); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};

protected:

private:
  CanFrameBT_509 data_;
  TimestampedCanFrame d_;
};


#endif
