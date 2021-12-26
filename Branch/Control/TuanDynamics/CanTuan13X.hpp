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

#ifndef __CanTUAN13x_h__
#define __CanTUAN13x_h__


#include "OpenICV/structure/structureCanTuan.h"


class  CanTuan13X
{
public:

  CanTuan13X(){};
  ~CanTuan13X(){}; 
  void decode(){

  data_.PLA_CRC=0;
  data_.PLA_BZ=0;
  data_.PLABrkRqtSt=0;
  data_.PLAExpStrWhlAngle=0;//期望方向盘转角
  data_.PLAExpStrWhlAngleDrt=0;
  data_.PLARequestStatus=0;
  data_.PLABrkTorque=0;
  data_.PLABrkDeceleration=0;
  data_.PLABrkEnable=0;
  data_.BrkTrqAndDeceSwt=0;
  data_.PLAParking=0;
  data_.PLAPrkDistance=0;
  data_.PLASignalTxCyclic=0;
  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2;

  temp=0;
  temp=(short)(pData[0]&0xFF);
  data_.PLA_CRC=temp;
 
  temp=0;
  temp=(short)(pData[1]&0x0F);
  data_.PLA_BZ=temp;

  temp=0;
  temp=(short)(pData[1]&0xF0);
  temp=temp>>4;
  data_.PLABrkRqtSt=temp;

  temp=0;
  temp=(short)(pData[2]&0xFF);
  temp1=0;
  temp1=(short)(pData[3]&0x1F);
  temp1=temp1<<8;
  temp1+=temp;
  data_.PLAExpStrWhlAngle=temp1*0.1;

  temp=0;
  temp=(short)(pData[3]&0x80);
  temp=temp>>7;
  data_.PLAExpStrWhlAngleDrt=temp;

  temp=0;
  temp=(short)(pData[4]&0x0F);
  data_.PLARequestStatus=temp;

  temp=0;
  temp=(short)(pData[4]&0xF0);
  temp=temp>>4;
  temp1=0;
  temp1=(short)(pData[5]&0xFF);
  temp1=temp1<<4;
  temp1+=temp;
  temp2=0;
  temp2=(short)(pData[5]&0x01);
  temp2=temp2<<12;
  temp2+=temp1;
  data_.PLABrkTorque=temp2*4;
  
  temp=0;
  temp=(short)(pData[4]&0xF0);
  temp=temp>>4;
  temp1=0;
  temp1=(short)(pData[5]&0x07);
  temp1=temp1<<4;
  temp1+=temp;
  data_.PLABrkDeceleration=temp1*0.1;

  temp=0;
  temp=(short)(pData[5]&0x08);
  temp=temp>>3;
  data_.PLABrkEnable=temp;

  temp=0;
  temp=(short)(pData[6]&0x04);
  temp=temp>>2;
  data_.BrkTrqAndDeceSwt=temp;

  temp=0;
  temp=(short)(pData[6]&0x08);
  temp=temp>>3;
  data_.PLAParking=temp;

  temp=0;
  temp=(short)(pData[6]&0xF0);
  temp=temp>>4;
  temp1=0;
  temp1=(short)(pData[7]&0x7F);
  temp1=temp1<<4;
  temp1+=temp;
  data_.PLAPrkDistance=temp1*0.01;

  temp=0;
  temp=(short)(pData[7]&0x80);
  temp=temp>>7;
  data_.PLASignalTxCyclic=temp;
  };
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameTouranPLA_01); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
  CanFrameTouranPLA_01 data_;
TimestampedCanFrame d_;
};



#endif

