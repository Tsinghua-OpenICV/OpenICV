/******************************************************************************
 * @file Can_SCU_IPC_3_0x206.hpp
 * @brief CAN  receive - SCU_IPC_3
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-07-13
 *****************************************************************************/

#ifndef __Can_SCU_IPC_3_0x206_H__
#define __Can_SCU_IPC_3_0x206_H__

#include "structureCanXP.h"

class  Can_SCU_IPC_3_0x206
{
public:

  Can_SCU_IPC_3_0x206(){};
  ~Can_SCU_IPC_3_0x206(){}; 
  void decode(){
  data_.DBWSt = 0;
  data_.VehSpdVD = 0; 
  data_.BrkPedalSt = 0; 
  data_.BrkPedalStVD = 0; 
  data_.VehSpd = 0; 
  data_.BrkLightOn = 0; 
  data_.MsgCounter = 0; 
  data_.Checksum = 0; 

  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;
  
  temp =0;
  temp=(short)((pData[0] >> 4)&0x0f);
  data_.DBWSt=temp;
  
  temp =0;
  temp=(short)((pData[0] >> 3)&0x01);
  data_.VehSpdVD=temp;

  temp =0;
  temp=(short)((pData[0] >> 2)&0x01);
  data_.BrkPedalSt=temp;

  temp =0;
  temp=(short)((pData[0] >> 1)&0x01);
  data_.BrkPedalStVD=temp;

  temp =0;
  temp1 =0;
  temp2 = 0;
  temp=(short)(pData[0]&0x01);
  temp1=(short)(pData[1]&0xff);
  temp2=(short)((pData[2] >> 4) &0x0f);
  temp=temp<<12;
  temp1=temp1<<4;
  temp2=temp2;
  data_.VehSpd = temp + temp1 + temp2;

  temp =0;
  temp=(short)((pData[2] >> 3)&0x01);
  data_.BrkLightOn=temp;

  temp =0;
  temp=(short)(pData[6]&0x0f);
  data_.MsgCounter = temp;

  temp =0;
  temp=(short)(pData[7]&0xff);
  data_.Checksum = temp;
  };    //end of decode()
  CanFrame_SCU_IPC_3_0x206* data() {return &data_; }; 
  int size() { return sizeof(CanFrame_SCU_IPC_3_0x206); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:

private:
  CanFrame_SCU_IPC_3_0x206 data_;
  TimestampedCanFrame d_;
};    //end of class  Can_SCU_IPC_3_0x206

#endif    //end of __Can_SCU_IPC_3_0x206_H__
