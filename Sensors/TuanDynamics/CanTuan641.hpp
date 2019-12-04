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

#ifndef __CanTUAN641_h__
#define __CanTUAN641_h__

#include "OpenICV/structure/structureCanTuan.h"


class  CanTuan641

{
public:


  CanTuan641(){};
  ~CanTuan641(){}; 
  void decode(){
  data_.EngTorqueCoefficient=0;
  uint8 *pData = d_.frame.data;
  int16 temp;

  temp=0;
  temp=(short)(pData[1]&0x30);
  temp=temp>>4;
  data_.EngTorqueCoefficient=temp;
	  
  };
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameTouranMotor_Code_01); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
  CanFrameTouranMotor_Code_01 data_;
TimestampedCanFrame d_;
};


#endif

