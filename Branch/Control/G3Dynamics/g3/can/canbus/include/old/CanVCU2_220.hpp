/******************************************************************************
 * @file CanVCU2_220.hpp
 * @brief CAN  receive - VCU_2
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-08
 *****************************************************************************/

#ifndef __CanVCU2_220_H__
#define __CanVCU2_220_H__

#include "structureCanXP.h"


class  CanVCU2_220
{
public:

  CanVCU2_220(){};
  ~CanVCU2_220(){}; 
  void decode(){
  data_.VCU2_TranCycle = 10;
  data_.MotorTorque = 0;

  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;
  
  temp =0;
  temp1 =0;
  temp=(short)(pData[2]&0xff);
  temp1=(short)(pData[3]&0xff);
  temp=temp<<8;
  data_.MotorTorque=temp+temp1;
  };    //end of decode()
  CanFrameVCU2_220* data() {return &data_; }; 
  int size() { return sizeof(CanFrameVCU2_220); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};

protected:

private:
  CanFrameVCU2_220 data_;
  TimestampedCanFrame d_;
};    //end of class  CanVCU2_220

#endif    //end of __CanVCU2_220_H__
