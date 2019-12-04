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

#ifndef __CANSPD_510_H__
#define __CANSPD_510_H__

#include "structureCanXP.h"

class  CanSPD_510
        
{
public:


  CanSPD_510(){};
  ~CanSPD_510(){};

  void decode(){

  data_.LF_WhlSpd=0;
  data_.LR_WhlSpd=0;
  data_.RF_WhlSpd=0;
  data_.RR_WhlSpd=0; 

  uint8 *pData = d_.frame.data;
  int16 temp,temp1;

  temp =0;
  temp1 =0;
  temp=(short)(pData[0]&0xff);
  temp1=(short)(pData[1]&0xff);
  temp=temp<<8;
  data_.LF_WhlSpd=temp1+temp;

  temp =0;
  temp1 =0;
  temp=(short)(pData[2]&0xff);
  temp1=(short)(pData[3]&0xff);
  temp=temp<<8;
  data_.RF_WhlSpd=temp1+temp;

  temp =0;
  temp1 =0;
  temp=(short)(pData[4]&0xff);
  temp1=(short)(pData[5]&0xff);
  temp=temp<<8;
  data_.LR_WhlSpd=temp1+temp;

  temp =0;
  temp1 =0;
  temp=(short)(pData[6]&0xff);
  temp1=(short)(pData[7]&0xff);
  temp=temp<<8;
  data_.RR_WhlSpd=temp1+temp;
};

  CanFrameSPD_510 * data() {return &data_; }; 
  int size() { return sizeof(CanFrameSPD_510); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};

  protected:

  private:
  CanFrameSPD_510 data_;
  TimestampedCanFrame d_;
};

#endif
