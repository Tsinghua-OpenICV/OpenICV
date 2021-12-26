/******************************************************************************
 * @file Can_SCU_IPC_6_0x209.hpp
 * @brief CAN  receive - SCU_IPC_6
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-07-13
 *****************************************************************************/

#ifndef __Can_SCU_IPC_6_0x209_H__
#define __Can_SCU_IPC_6_0x209_H__

#include "structureCanXP.h"

class  Can_SCU_IPC_6_0x209
{
public:

  Can_SCU_IPC_6_0x209(){};
  ~Can_SCU_IPC_6_0x209(){}; 
  void decode(){
  data_.dstBat_Dsp = 0;
  data_.AccPedalSig = 0; 
  data_.BrkPedalSt = 0; 
  data_.BrkPedalStVD = 0; 
  data_.CurrentGearLevVD = 0; 
  data_.CurrentGearLev = 0; 
  data_.MsgCounter = 0; 
  data_.Checksum = 0; 

  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;
  
  temp =0;
  temp1 =0;
  temp=(short)(pData[0]&0xff);
  temp1=(short)((pData[1] >> 6) & 0x03);
  temp=temp<<2;
  data_.dstBat_Dsp=temp+temp1;
  
  temp =0;
  temp1 =0;
  temp=(short)(pData[1]&0x3f);
  temp1=(short)((pData[2] >> 6) & 0x03);
  temp=temp<<2;
  data_.AccPedalSig=temp+temp1;

  temp =0;
  temp=(short)((pData[2]  >> 5) & 0x01);
  data_.BrkPedalSt=temp;

  temp =0;
  temp=(short)((pData[2]  >> 4) & 0x01);
  data_.BrkPedalStVD=temp;

  temp =0;
  temp=(short)((pData[2]  >> 3) & 0x01);
  data_.CurrentGearLevVD=temp;

  temp =0;
  temp=(short)(pData[2] & 0x07);
  data_.CurrentGearLev=temp;
  
  temp =0;
  temp=(short)(pData[6]&0x0f);
  data_.MsgCounter = temp;

  temp =0;
  temp=(short)(pData[7]&0xff);
  data_.Checksum = temp;
  };    //end of decode()
  CanFrame_SCU_IPC_6_0x209* data() {return &data_; }; 
  int size() { return sizeof(CanFrame_SCU_IPC_6_0x209); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:

private:
  CanFrame_SCU_IPC_6_0x209 data_;
  TimestampedCanFrame d_;
};    //end of class  Can_SCU_IPC_6_0x209

#endif    //end of __Can_SCU_IPC_6_0x209_H__
