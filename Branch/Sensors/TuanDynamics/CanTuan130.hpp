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

#ifndef __CanTUAN130_h__
#define __CanTUAN130_h__


#include "OpenICV/structure/structureCanTuan.h"


class  CanTuan130
{
public:

  CanTuan130(){};
  ~CanTuan130(){}; 
  void decode(){

  data_.EPBFaultStatus=0;
  data_.EPBSwitch=0;
  data_.EPBSwitchState=0;
  data_.EPB_PressingForce=0;
  data_.EPB_Status=0;
  uint8 *pData = d_.frame.data;
  int16 temp,temp1;
  
  temp=0;
  temp=(short)(pData[6]&0xc);
  temp=temp>>2;
  data_.EPBFaultStatus=temp;

  temp=0;
  temp=(short)(pData[6]&0x30);
  temp=temp>>4;
  data_.EPBSwitch=temp;

  temp=0;
  temp=(short)(pData[6]&0x40);
  temp=temp>>6;
  data_.EPBSwitchState=temp;

  temp=0;
  temp=(short)(pData[7]&0x1f);
  data_.EPB_PressingForce=temp;

  temp=0;
  temp=(short)(pData[7]&0x60);
  temp=temp>>5;
  data_.EPB_Status=temp;
  };
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameTouranEPB_01); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
  CanFrameTouranEPB_01 data_;
TimestampedCanFrame d_;
};



#endif

