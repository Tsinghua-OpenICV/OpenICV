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

#ifndef __CanTuan30C_h__
#define __CanTuan30C_h__

#include "OpenICV/structure/structureCanTuan.h"



class CanTuan30C
        
{
public:

  CanTuan30C(){};
  ~CanTuan30C(){}; 
  void decode(){

  data_.ACC_02CRC=0;
  data_.ACC_02BZ=0; 
  data_.ExpectedSpeed=0; 
  data_.SpacingFactor=0;  
  data_.NoiseAlarm=0; 
  data_.SetSpacing=0;
  data_.ImageAlarm=0; 
  data_.ACC02Status=0; 
  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;

  temp =0;
  temp=(short)(pData[0]&0xff);
  data_.ACC_02CRC=temp;

  temp =0;
  temp=(short)(pData[1]&0xf);
  data_.ACC_02BZ=temp;

  temp=0;
  temp1=0;
  temp=(short)(pData[1]&0xf0);
  temp=temp>>4;
  temp1=(short)(pData[2]&0x3f);
  temp1=temp1<<4;
  temp1+=temp;
  data_.ExpectedSpeed=temp1*0.32; 

  temp=0;
  temp1=0;
  temp=(short)(pData[3]&0xff);
  temp1=(short)(pData[4]&0x3);
  temp1=temp1<<8;
  temp1+=temp;
  data_.SpacingFactor=temp1*0.32;

  temp =0;
  temp=(short)(pData[4]&0xc);
  temp=temp>>2;
  data_.NoiseAlarm=temp;

  temp =0;
  temp=(short)(pData[4]&0xE0);
  temp=temp>>5;
  data_.SetSpacing=temp;

  temp =0;
  temp=(short)(pData[5]&0x1);
  data_.ImageAlarm=temp;

  temp =0;
  temp=(short)(pData[7]&0xE0);
  temp=temp>>5;
  data_.ACC02Status=temp;   
  };
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameTouranACC_02); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
  CanFrameTouranACC_02 data_;
TimestampedCanFrame d_;
};


#endif

