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

#ifndef __CanTuan0AD_h__
#define __CanTuan0AD_h__

#include "OpenICV/structure/structureCanTuan.h"



class CanTuan0AD
        
{
public:

  CanTuan0AD(){};
  ~CanTuan0AD(){}; 
  void decode(){
  data_.Stalls=0;
  data_.TargetStalls=0;
  uint8 *pData = d_.frame.data;
  int16 temp;
  
  temp=0;
  temp=(short)(pData[5]&0x3C);
  temp=temp>>2;
  data_.Stalls=temp;
  
  temp=0;
  temp=(short)(pData[7]&0xF0);
  temp=temp>>4;
  data_.TargetStalls=temp;
  };
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameGear_11); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
  CanFrameGear_11 data_;
TimestampedCanFrame d_;
};


#endif

