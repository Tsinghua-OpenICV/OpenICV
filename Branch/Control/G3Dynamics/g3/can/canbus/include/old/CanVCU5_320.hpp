/******************************************************************************
 * @file CanVCU5_320.hpp
 * @brief CAN  receive - VCU_5
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-08
 *****************************************************************************/

#ifndef __CanVCU5_320_H__
#define __CanVCU5_320_H__

#include "structureCanXP.h"


class  CanVCU5_320
{
public:
  CanVCU5_320(){};
  ~CanVCU5_320(){}; 
  void decode(){
  data_.VCU5_TranCycle = 100;
  data_.SOC = 0;

  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;
  
  temp =0;
  temp=(short)(pData[4]&0xff);
  data_.SOC=temp;
  };    //end of decode()
  CanFrameVCU5_320* data() {return &data_; }; 
  int size() { return sizeof(CanFrameVCU5_320); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};

protected:

private:
  CanFrameVCU5_320 data_;
  TimestampedCanFrame d_;
};    //end of class  CanVCU5_320

#endif    //end of __CanVCU5_320_H__
