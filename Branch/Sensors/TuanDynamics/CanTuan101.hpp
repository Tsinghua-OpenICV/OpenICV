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

#ifndef __CanTUAN101_h__
#define __CanTUAN101_h__


#include "OpenICV/structure/structureCanTuan.h"


class  CanTuan101
{
public:

  CanTuan101(){};
  ~CanTuan101(){}; 
  void decode(){



  data_.YawState=0;
  data_.YawRate=0; 
  data_.YawToRight=0; 
  data_.axState=0;  
  data_.ax=0; 
  data_.ayState=0;
  data_.ay=0; 
  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;

  temp =0;
  temp=(short)(pData[1]&0x40);
  temp=temp>>6;
  data_.ayState=temp;

  temp =0;
  temp=(short)(pData[1]&0x20);
  temp=temp>>5;
  data_.axState=temp;

  temp =0;
  temp=(short)(pData[1]&0x10);
  temp=temp>>4;
  data_.YawState=temp;

  temp =0;
  temp=(short)(pData[2]&0xff);
  data_.ay=temp*0.01-1.27;

  temp=0;
  temp1=0;
  temp=(short)(pData[3]&0xff);
  temp1=(short)(pData[4]&0x3);
  temp1=temp1<<8;
  temp1+=temp;
  data_.ax=temp1*0.03125-16; 

  temp=0;
  temp1=0;
  temp=(short)(pData[6]&0xff);
  temp1=(short)(pData[7]&0x3F);
  temp1=temp1<<8;
  temp1+=temp;
  data_.YawRate=temp1*0.01;

  temp =0;
  temp=(short)(pData[6]&0x40);
  temp=temp>>6;
  data_.YawToRight=temp;
  };
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameTuanESP_02); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
  CanFrameTuanESP_02 data_;
TimestampedCanFrame d_;
};



#endif

