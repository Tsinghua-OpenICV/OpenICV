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

#ifndef __CanTUAN106_h__
#define __CanTUAN106_h__

#include "OpenICV/structure/structureCanTuan.h"



class  CanTuan106
{
public:

  CanTuan106(){};
  ~CanTuan106(){}; 
  void decode(){
  data_.BrakePressureState=0;
  data_.BrakePressure=0; 
  uint8 *pData = d_.frame.data;
   int16 temp,temp1,temp2,temp3;
   
   temp =0;
   temp=(short)(pData[1]&0x10);
   temp=temp>>4;
   data_.BrakePressureState=temp;
   
   temp=0;
   temp1=0;
   temp=(short)(pData[2]&0xff);
   temp1=(short)(pData[3]&0x3F);
   temp1=temp1<<8;
   temp1+=temp;
   data_.BrakePressure=temp1*0.3-30;
  };
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameTuanESP_05); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
  CanFrameTuanESP_05 data_;
TimestampedCanFrame d_;
};


#endif

