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

#ifndef __CanTUAN12E_h__
#define __CanTUAN12E_h__


#include "OpenICV/structure/structureCanTuan.h"


class  CanTuan12E
{
public:

  CanTuan12E(){};
  ~CanTuan12E(){}; 
  void decode(){

data_.ACC_07CRC=0;
  data_.ACC_07BZ=0; 
  data_.ParkingDistance=0; 
  data_.ParkingRequest07=0;  
  data_.StartRequest07=0; 
  data_.aExpValue=0;
  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;

  temp =0;
  temp=(short)(pData[0]&0xff);
  data_.ACC_07CRC=temp;

  temp =0;
  temp=(short)(pData[1]&0xf);
  data_.ACC_07BZ=temp;

  temp=0;
  temp1=0;
  temp=(short)(pData[1]&0xf0);
  temp=temp>>4;
  temp1=(short)(pData[2]&0x7F);
  temp1=temp1<<4;
  temp1+=temp;
  data_.ParkingDistance=temp1*0.01; 

  temp =0;
  temp=(short)(pData[2]&0x80);
  temp=temp>>7;
  data_.ParkingRequest07=temp;

  temp =0;
  temp=(short)(pData[3]&0x80);
  temp=temp>>7;
  data_.StartRequest07=temp;   

  temp=0;
  temp1=0;
  temp=(short)(pData[6]&0xE0);
  temp=temp>>5;
  temp1=(short)(pData[7]&0xFF);
  temp1=temp1<<3;
  temp1+=temp;
  data_.aExpValue=temp1*0.005-7.22;

  };
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameTouranACC_07); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
  CanFrameTouranACC_07 data_;
TimestampedCanFrame d_;
};



#endif

