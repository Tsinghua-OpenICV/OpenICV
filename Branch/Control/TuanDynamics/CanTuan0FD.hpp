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

#ifndef __CanTUAN0FD_h__
#define __CanTUAN0FD_h__

#include "OpenICV/structure/structureCanTuan.h"



class CanTuan0FD
        
{
public:

  CanTuan0FD(){};
  ~CanTuan0FD(){}; 
  void decode(){
  data_.Speed=0;
  data_.ESP_SystemStatus=0;
  data_.SpeedState=0;
  uint8 *pData = d_.frame.data;
  int16 temp,temp1;

  temp=0;
  temp=(short)(pData[4]&0xFF);
  temp1=0;
  temp1=(short)(pData[5]&0xFF);
  temp1=temp1<<8;
  temp1+=temp;
  data_.Speed=temp1*0.01;

  temp=0;
  temp=(short)(pData[6]&0x04);
  temp1=temp>>2;
  data_.ESP_SystemStatus=temp;

  temp=0;
  temp=(short)(pData[6]&0x80);
  temp1=temp>>7;
  data_.SpeedState=temp;
  };
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameTuanESP_21); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
  CanFrameTuanESP_21 data_;
TimestampedCanFrame d_;
};


#endif

