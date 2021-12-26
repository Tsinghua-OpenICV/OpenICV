/******************************************************************************
 * @file Can_SCU_IPC_2_0x205.hpp
 * @brief CAN  receive - SCU_IPC_2
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-07-13
 *****************************************************************************/

#ifndef __Can_SCU_IPC_2_0x205_H__
#define __Can_SCU_IPC_2_0x205_H__

#include "structureCanXP.h"

class  Can_SCU_IPC_2_0x205
{
public:

  Can_SCU_IPC_2_0x205(){};
  ~Can_SCU_IPC_2_0x205(){}; 
  void decode(){
  data_.FLWheelSpd = 0;
  data_.FRWheelSpd = 0; 
  data_.RLWheelSpd = 0; 
  data_.RRWheelSpd = 0; 
  data_.FLWheelSpdVD = 0; 
  data_.FRWheelSpdVD = 0; 
  data_.RLWheelSpdVD = 0; 
  data_.RRWheelSpdVD = 0; 

  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;
  
  temp =0;
  temp1 =0;
  temp=(short)(pData[0]&0xff);
  temp1=(short)((pData[1] >> 3) & 0x1f);
  temp=temp<<5;
  data_.FLWheelSpd=temp+temp1;

  temp =0;
  temp1 =0;
  temp2 = 0;
  temp=(short)(pData[1]&0x03);
  temp1=(short)(pData[2]&0xff);
  temp2=(short)((pData[3] >> 6) &0x03);
  temp=temp<<10;
  temp1=temp1<<2;
  temp2=temp2;
  data_.FRWheelSpd=temp+temp1 + temp2;

  temp =0;
  temp1 =0;
  temp=(short)(pData[3]&0x3f);
  temp1=(short)((pData[4] >> 1) & 0x7f);
  temp=temp<<7;
  data_.RLWheelSpd=temp+temp1;

  temp =0;
  temp1 =0;
  temp2 = 0;
  temp=(short)(pData[4]&0x01);
  temp1=(short)(pData[5]&0xff);
  temp2=(short)((pData[6] >> 4) &0x0f);
  temp=temp<<12;
  temp1=temp1<<4;
  temp2=temp2;
  data_.RRWheelSpd=temp+temp1 + temp2;

  temp =0;
  temp=(short)((pData[6]  >> 3) & 0x01);
  data_.FLWheelSpdVD=temp;

  temp =0;
  temp=(short)((pData[6]  >> 2) & 0x01);
  data_.FRWheelSpdVD=temp;

  temp =0;
  temp=(short)((pData[6]  >> 1) & 0x01);
  data_.RRWheelSpdVD=temp;

  temp =0;
  temp=(short)(pData[6] & 0x01);
  data_.RLWheelSpdVD=temp;
  };    //end of decode()
  CanFrame_SCU_IPC_2_0x205* data() {return &data_; }; 
  int size() { return sizeof(CanFrame_SCU_IPC_2_0x205); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:

private:
  CanFrame_SCU_IPC_2_0x205 data_;
  TimestampedCanFrame d_;
};    //end of class  Can_SCU_IPC_2_0x205

#endif    //end of __Can_SCU_IPC_2_0x205_H__
