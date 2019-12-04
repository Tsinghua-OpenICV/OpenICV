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

#ifndef __CanTUAN116_h__
#define __CanTUAN116_h__

#include "OpenICV/structure/structureCanTuan.h"



class  CanTuan116
        
{
public:


  CanTuan116(){};
  ~CanTuan116(){};//	typedef  struct
//{
//    bool LeftFrontDir;         ///< 
//	bool RightFrontDir;
//	bool LeftRearDir;
//	bool RightRearDir;
//
//
//} CanFrameTuanESP_10;//0x116;

  void decode(){

data_.LF_WhlSpdDrt=0;
  data_.LR_WhlSpdDrt=0;
  data_.RF_WhlSpdDrt=0;
  data_.RR_WhlSpdDrt=0;  
  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;

  temp =0;
  temp=(short)(pData[7]&0x3);
  data_.LF_WhlSpdDrt=temp;

  temp =0;
  temp=(short)(pData[7]&0xC);
  temp=temp>>2;
  data_.RF_WhlSpdDrt=temp;

  temp =0;
  temp=(short)(pData[7]&0x30);
  temp=temp>>4;
  data_.LR_WhlSpdDrt=temp;

  temp =0;
  temp=(short)(pData[7]&0xC0);
  temp=temp>>6;
  data_.RR_WhlSpdDrt=temp;
}
;
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameTuanESP_10); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
  CanFrameTuanESP_10 data_;
TimestampedCanFrame d_;
};



#endif

