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

#ifndef __CanTuanAD_h__
#define __CanTuanAD_h__

#include "OpenICV/structure/structureCanTuan.h"



class CanTuan1AB
        
{
public:

  CanTuan1AB(){};
  ~CanTuan1AB(){}; 
  void decode(){

  data_.ACCSignalContinuity=0;
  uint8 *pData = d_.frame.data;
  int16 temp;
  
  temp=0;
  temp=(short)(pData[4]&0x40);
  temp=temp>>6;
  data_.ACCSignalContinuity=temp;
  };
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameTuanESP_33); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
  CanFrameTuanESP_33 data_;
TimestampedCanFrame d_;
};


#endif

