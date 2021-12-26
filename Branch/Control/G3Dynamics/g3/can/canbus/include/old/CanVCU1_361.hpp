/******************************************************************************
 * @file CanVCU1_361.hpp
 * @brief CAN  receive - VCU_1
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-08
 *****************************************************************************/

#ifndef __CanVCU1_361_H__
#define __CanVCU1_361_H__

#include "structureCanXP.h"


class  CanVCU1_361
{
public:

  CanVCU1_361(){};
  ~CanVCU1_361(){}; 
  void decode(){
  data_.VCU1_TranCycle = 100;
  data_.AccAppPosition = 0; 
  data_.DriveModeState = 0; 

  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;
  
  temp =0;
  temp=(short)(pData[1]&0xff);
  data_.AccAppPosition=temp;

  temp =0;
  temp=(short)(pData[2]&0x01);
  data_.DriveModeState=temp;
  };    //end of decode()
  CanFrameVCU1_361* data() {return &data_; }; 
  int size() { return sizeof(CanFrameVCU1_361); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};

protected:

private:
  CanFrameVCU1_361 data_;
  TimestampedCanFrame d_;
};    //end of class  CanVCU1_361

#endif    //end of __CanVCU1_361_H__
