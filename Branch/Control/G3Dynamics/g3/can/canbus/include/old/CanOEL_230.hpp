/******************************************************************************
 * @file CanOEL_230.hpp
 * @brief CAN  receive - OEL
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-08
 *****************************************************************************/

#ifndef __CanOEL_230_H__
#define __CanOEL_230_H__

#include "structureCanXP.h"


class  CanOEL_230
{
public:

  CanOEL_230(){};
  ~CanOEL_230(){}; 
  void decode(){
  data_.OEL_TranCycle = 100;
  data_.TurnLightSw = 0; 
  data_.HazardSw = 0; 
  
  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;
  
  temp =0;
  temp=(short)(pData[1]&0x0f);
  data_.TurnLightSw=temp;

  temp =0;
  temp=(short)(pData[1]&0x30);
  temp=temp>>4;
  data_.HazardSw=temp;
  };    //end of decode()
  CanFrameOEL_230* data() {return &data_; }; 
  int size() { return sizeof(CanFrameOEL_230); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};

protected:

private:
  CanFrameOEL_230 data_;
  TimestampedCanFrame d_;
};    //end of class  CanOEL_230

#endif    //end of __CanOEL_230_H__
