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

#ifndef __CanTUAN122_h__
#define __CanTUAN122_h__


#include "OpenICV/structure/structureCanTuan.h"


class  CanTuan122
{
public:

  CanTuan122(){};
  ~CanTuan122(){}; 
  void decode(){

data_.ACC_06CRC=0;
  data_.ACC_06BZ=0; 
  data_.aLowerDeviation=0; 
  data_.aExpectedValue=0;  
  data_.aUpperDeviation=0; 
  data_.DclrtGradRmdValue=0;
  data_.AclrtGradRmdValue=0; 
  data_.StartRequest06=0; 
  data_.ParkingRequest06=0; 
  data_.ACC06Status=0; 
  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;

  temp =0;
  temp=(short)(pData[0]&0xff);
  data_.ACC_06CRC=temp;

  temp =0;
  temp=(short)(pData[1]&0xf);
  data_.ACC_06BZ=temp;

  temp =0;
  temp=(short)(pData[2]&0x3f);
  data_.aLowerDeviation=temp*0.024;

  temp=0;
  temp1=0;
  temp=(short)(pData[3]&0xff);
  temp1=(short)(pData[2]&0x7);
  temp1=temp1<<8;
  temp1+=temp;
  data_.aExpectedValue=temp1*0.005-7.22; 

  temp =0;
  temp=(short)(pData[4]&0xF8);
  temp=temp>>3;
  data_.aUpperDeviation=temp*0.0625;   

  temp =0;
  temp=(short)(pData[5]&0xff);
  data_.DclrtGradRmdValue=temp*0.05;

  temp =0;
  temp=(short)(pData[6]&0xff);
  data_.AclrtGradRmdValue=temp*0.05;

  temp =0;
  temp=(short)(pData[7]&0x1);
  data_.StartRequest06=temp;

  temp =0;
  temp=(short)(pData[7]&0x2);
  temp=temp>>1;
  data_.ParkingRequest06=temp;

  temp =0;
  temp=(short)(pData[7]&0x70);//modified by wwu
  temp=temp>>4;
  data_.ACC06Status=temp;
  };
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameTouranACC_06); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
  CanFrameTouranACC_06 data_;
TimestampedCanFrame d_;
};



#endif

