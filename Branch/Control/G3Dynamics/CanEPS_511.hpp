//
//  created:    2017/12/15
//
//  author:     Kun JIANG
//              Copyright Tsinghua University
// 
//  version:    $Id: $
//
//  purpose:    
//

#ifndef __CANEPS_511_H__
#define __CANEPS_511_H__

#include "structureCanXP.h"


class  CanEPS_511
{
public:

  CanEPS_511(){};
  ~CanEPS_511(){}; 
  void decode(){

  data_.EPS_angle_spd_ccp=0; 
  data_.EPS_angle_ccp=0; 
  data_.EPS_StrngWhlTorq=0; 


   uint8 *pData = d_.frame.data;
   int16 temp,temp1,temp2,temp3;
   
  temp =0;
  temp1 =0;
  temp=(short)(pData[0]&0xff);
  temp1=(short)(pData[1]&0xff);
  temp=temp<<8;
  data_.EPS_angle_spd_ccp=temp1+temp;

  temp =0;
  temp1 =0;
  temp=(short)(pData[2]&0xff);
  temp1=(short)(pData[3]&0xff);
  temp=temp<<8;
  data_.EPS_angle_ccp=temp1+temp;

  temp =0;
  temp1 =0;
  temp=(short)(pData[4]&0xff);
  temp1=(short)(pData[5]&0xff);
  temp=temp<<8;
  data_.EPS_StrngWhlTorq=temp1+temp;


  };
  CanFrameEPS_511 * data() {return &data_; }; 
  int size() { return sizeof(CanFrameEPS_511); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};

protected:

private:
  CanFrameEPS_511 data_;
  TimestampedCanFrame d_;
};


#endif


