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

#ifndef __CanTUAN09F_h__
#define __CanTUAN09F_h__

#include "OpenICV/structure/structureCanTuan.h"



class CanTuan09F
        
{
public:

  CanTuan09F(){};
  ~CanTuan09F(){}; 
  void decode(){
  data_.EPSRxHCA_Status=0;
  data_.EPS_StrWhlTorque=0;
  data_.EPS_StrWhlTorqueDrt=0;
  data_.EPS_StrWhlTorqueSt=0;

  uint8 *pData = d_.frame.data;
  int16 temp,temp1;

  temp=0;
  temp=(short)(pData[4]&0x0F);
  data_.EPSRxHCA_Status=temp;

  temp=0;
  temp=(short)(pData[5]&0xFF);
  temp1=0;
  temp1=(short)(pData[6]&0x03);
  temp1=temp1<<8;
  temp1+=temp;
  data_.EPS_StrWhlTorque=temp1*0.01;

  temp=0;
  temp=(short)(pData[6]&0x80);
  temp=temp>>7;
  data_.EPS_StrWhlTorqueDrt=temp;

  temp=0;
  temp=(short)(pData[6]&0x40);
  temp=temp>>6;
  data_.EPS_StrWhlTorqueSt=temp;
  };
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameTouranLH_EPS_03); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
  CanFrameTouranLH_EPS_03 data_;
TimestampedCanFrame d_;
};


#endif

