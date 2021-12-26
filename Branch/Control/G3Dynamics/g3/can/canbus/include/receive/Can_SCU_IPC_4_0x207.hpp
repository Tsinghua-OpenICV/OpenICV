/******************************************************************************
 * @file Can_SCU_IPC_4_0x207.hpp
 * @brief CAN  receive - SCU_IPC_4
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-07-13
 *****************************************************************************/

#ifndef __Can_SCU_IPC_4_0x207_H__
#define __Can_SCU_IPC_4_0x207_H__

#include "structureCanXP.h"

class  Can_SCU_IPC_4_0x207
{
public:
  Can_SCU_IPC_4_0x207(){};
  ~Can_SCU_IPC_4_0x207(){}; 
  void decode(){
  data_.ActVehLongAccelVD = 0;
  data_.ActVehLateralAccelVD = 0; 
  data_.YAWVD = 0; 
  data_.YAW = 0; 
  data_.ActVehLongAccel = 0; 
  data_.ActVehLateralAccel = 0; 
  data_.EPBSysSt = 0;   
  data_.MsgCounter = 0; 
  data_.Checksum = 0; 

  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;
  
  temp =0;
  temp=(short)((pData[0] >> 7)&0x01);
  data_.ActVehLongAccelVD=temp;
  
  temp =0;
  temp=(short)((pData[0] >> 6)&0x01);
  data_.ActVehLateralAccelVD=temp;

  temp =0;
  temp=(short)((pData[0] >> 5)&0x01);
  data_.YAWVD=temp;

  temp =0;
  temp1 =0;
  temp=(short)(pData[0]&0x1f);
  temp1=(short)((pData[1] >> 1) & 0x7f);
  temp=temp<<7;
  data_.YAW=temp+temp1;

  temp =0;
  temp1 =0;
  temp2 = 0;
  temp=(short)(pData[1]&0x01);
  temp1=(short)(pData[2]&0xff);
  temp2=(short)((pData[3] >> 5) &0x07);
  temp=temp<<11;
  temp1=temp1<<3;
  temp2=temp2;
  data_.ActVehLongAccel = temp + temp1 + temp2;

  temp =0;
  temp1 =0;
  temp=(short)(pData[3]&0x1f);
  temp1=(short)((pData[4] >> 1) & 0x7f);
  temp=temp<<7;
  data_.ActVehLateralAccel=temp+temp1;

  temp =0;
  temp1 =0;
  temp=(short)(pData[4]&0x01);
  temp1=(short)((pData[5] >> 6) & 0x03);
  temp=temp<<2;
  data_.EPBSysSt=temp+temp1;

  temp =0;
  temp=(short)(pData[6]&0x0f);
  data_.MsgCounter = temp;

  temp =0;
  temp=(short)(pData[7]&0xff);
  data_.Checksum = temp;
  };    //end of decode()
  CanFrame_SCU_IPC_4_0x207* data() {return &data_; }; 
  int size() { return sizeof(CanFrame_SCU_IPC_4_0x207); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:

private:
  CanFrame_SCU_IPC_4_0x207 data_;
  TimestampedCanFrame d_;
};    //end of class  Can_SCU_IPC_4_0x207

#endif    //end of __Can_SCU_IPC_4_0x207_H__
