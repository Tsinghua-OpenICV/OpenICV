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

#ifndef __CanTUAN126_h__
#define __CanTUAN126_h__


#include "OpenICV/structure/structureCanTuan.h"


class  CanTuan126
{
public:

  CanTuan126(){};
  ~CanTuan126(){}; 
  void decode(){

  data_.ExpStrWhlTorque=0;
  data_.HCATranCycle=0;
  data_.ExpStrWhlTorqueDrt=0;
  data_.HCA_Status=0;
  uint8 *pData = d_.frame.data;
  int16 temp,temp1;

  temp=0;
  temp=(short)(pData[2]&0xFF);
  temp1=0;
  temp1=(short)(pData[3]&0x01);
  temp1=temp1<<8;
  temp1+=temp;
  data_.ExpStrWhlTorque=temp1*0.01;


  temp=0;
  temp=(short)(pData[3]&0x40);
  temp=temp>>6;
  data_.HCATranCycle=temp;

  temp=0;
  temp=(short)(pData[3]&0x80);
  temp=temp>>7;
  data_.ExpStrWhlTorqueDrt=temp;

  temp=0;
  temp=(short)(pData[4]&0x0F);
  data_.HCA_Status=temp;
  };
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameTouranHCA_01); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
  CanFrameTouranHCA_01 data_;
TimestampedCanFrame d_;
};



#endif

