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

#ifndef __CANCOM_5F0_H__
#define __CANCOM_5F0_H__

#include "structureCanXP.h"



class CanCOM_5F0
        
{
public:

  CanCOM_5F0(){};
  ~CanCOM_5F0(){}; 
  void decode(){


  data_.Com_AutoMode=0; 
  data_.Com_VoiceAlarm=0; 
  data_.SCU_TarSpeed_Req=0;
  data_.SCU_EPSAngle_Req=0; 
  data_.Com_TurnLight=0;  


  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;

  temp =0;
  temp=(short)(pData[0]&0x0f);
  data_.Com_AutoMode=temp;

  temp =0;
  temp=(short)(pData[0]&0x30);
  temp=temp>>4;
  data_.Com_VoiceAlarm=temp;

  temp=0;
  temp1=0;
  temp=(short)(pData[1]&0xff);
  temp1=(short)(pData[2]&0xff);
  temp1=temp1<<8;
  temp1+=temp;
  data_.SCU_TarSpeed_Req=temp1; 

  temp=0;
  temp1=0;
  temp=(short)(pData[5]&0xff);
  temp1=(short)(pData[6]&0xff);
  temp1=temp1<<8;
  temp1+=temp;
  data_.SCU_EPSAngle_Req=temp1;

  temp =0;
  temp=(short)(pData[7]&0xff);
  data_.Com_TurnLight=temp;

  };
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameCOM_5F0); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};

protected:

private:
  CanFrameCOM_5F0 data_;
  TimestampedCanFrame d_;
};


#endif

