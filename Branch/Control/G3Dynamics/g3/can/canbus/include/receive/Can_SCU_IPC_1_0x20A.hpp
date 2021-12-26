/******************************************************************************
 * @file Can_SCU_IPC_1_0x20A.hpp
 * @brief CAN  receive - SCU_IPC_1
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-07-13
 *****************************************************************************/

#ifndef __Can_SCU_IPC_1_0x20A_H__
#define __Can_SCU_IPC_1_0x20A_H__

#include "structureCanXP.h"

class  Can_SCU_IPC_1_0x20A
{
public:

  Can_SCU_IPC_1_0x20A(){};
  ~Can_SCU_IPC_1_0x20A(){}; 
  void decode(){
  data_.SteeringAngleVD = 0;
  data_.SteeringAngleSpd = 0; 
  data_.SteeringAngle = -780; 
  data_.ResponseTorque = 0; 
  data_.ResponseTorqueVD = 0; 
  data_.SteeringAngleSpdVD = 0; 
  data_.MsgCounter = 0; 
  data_.Checksum = 0; 

  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;
  
  temp =0;
  temp=(short)((pData[0] >> 7)&0x01);
  data_.SteeringAngleVD=temp;
  
  temp =0;
  temp1 =0;
  temp=(short)(pData[0]&0x7f);
  temp1=(short)((pData[1] >> 5) & 0x07);
  temp=temp<<3;
  data_.SteeringAngleSpd=temp+temp1;

  temp =0;
  temp1 =0;
  temp2 = 0;
  temp=(short)(pData[1]&0x1f);
  temp1=(short)(pData[2]&0xff);
  temp2=(short)((pData[3] >> 5) &0x07);
  temp=temp<<11;
  temp1=temp1<<3;
  temp2=temp2;
  data_.SteeringAngle = temp + temp1 + temp2;

  temp =0;
  temp1 =0;
  temp=(short)(pData[3]&0x1f);
  temp1=(short)((pData[4] >> 2) & 0x3f);
  temp=temp<<6;
  data_.ResponseTorque=temp+temp1;

  temp =0;
  temp=(short)((pData[4]  >> 1) & 0x01);
  data_.ResponseTorqueVD=temp;

  temp =0;
  temp=(short)(pData[4] & 0x01);
  data_.SteeringAngleSpdVD=temp;

  temp =0;
  temp=(short)(pData[6]&0x0f);
  data_.MsgCounter = temp;

  temp =0;
  temp=(short)(pData[7]&0xff);
  data_.Checksum = temp;
  };    //end of decode()
  CanFrame_SCU_IPC_1_0x20A* data() {return &data_; }; 
  int size() { return sizeof(CanFrame_SCU_IPC_1_0x20A); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:

private:
  CanFrame_SCU_IPC_1_0x20A data_;
  TimestampedCanFrame d_;
};    //end of class  Can_SCU_IPC_1_0x20A

#endif    //end of __Can_SCU_IPC_1_0x20A_H__
