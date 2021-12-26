/******************************************************************************
 * @file Can_SCU_IPC_7_0x301.hpp
 * @brief CAN  receive - SCU_IPC_7
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-07-13
 *****************************************************************************/

#ifndef __Can_SCU_IPC_7_0x301_H__
#define __Can_SCU_IPC_7_0x301_H__

#include "structureCanXP.h"

class  Can_SCU_IPC_7_0x301
{
public:

  Can_SCU_IPC_7_0x301(){};
  ~Can_SCU_IPC_7_0x301(){}; 
  void decode(){
  data_.PSeatBeltWarning = 0;
  data_.DriverSeatBeltWarning = 0; 
  data_._1ndLPSeatBeltWar = 0; 
  data_._1ndMPSeatBeltWar = 0; 
  data_._1ndRPSeatBeltWar = 0; 
  data_.TotalOdometerVD = 0; 
  data_.TotalOdometer = 0; 
  data_.ACSt = 0; 
  data_.ErrSt = 0; 
  data_.OverrideRes = 0; 

  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;

  temp =0;
  temp=(short)((pData[0]  >> 2) & 0x01);
  data_.PSeatBeltWarning=temp;

  temp =0;
  temp=(short)((pData[0]  >> 3) & 0x01);
  data_.DriverSeatBeltWarning=temp;  

  temp =0;
  temp=(short)((pData[0]  >> 4) & 0x01);
  data_._1ndLPSeatBeltWar=temp;

  temp =0;
  temp=(short)((pData[0]  >> 5) & 0x01);
  data_._1ndMPSeatBeltWar=temp;

  temp =0;
  temp=(short)((pData[0]  >> 6) & 0x01);
  data_._1ndRPSeatBeltWar=temp;

  temp =0;
  temp=(short)((pData[0]  >> 7) & 0x01);
  data_.TotalOdometerVD=temp;
  
  temp =0;
  temp1 =0;
  temp2 = 0;
  temp3 = 0;
  temp=(short)(pData[0]&0x03);
  temp1=(short)(pData[1]&0xff);
  temp2=(short)(pData[2]&0xff);
  temp3=(short)((pData[3]  >> 6)&0x03);
  temp=temp<<18;
  temp1=temp1<<10;
  temp2=temp2<<2;
  data_.TotalOdometer=temp + temp1 + temp2 + temp3;

  temp =0;
  temp=(short)((pData[3]  >> 5) & 0x01);
  data_.ACSt=temp;

  temp =0;
  temp=(short)((pData[3]  >> 1) & 0x0f);
  data_.ErrSt=temp;

  temp =0;
  temp1 =0;
  temp=(short)(pData[3]&0x01);
  temp1=(short)((pData[4] >> 5) & 0x07);
  temp=temp<<3;
  data_.OverrideRes=temp+temp1;
  };    //end of decode()
  CanFrame_SCU_IPC_7_0x301* data() {return &data_; }; 
  int size() { return sizeof(CanFrame_SCU_IPC_7_0x301); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:

private:
  CanFrame_SCU_IPC_7_0x301 data_;
  TimestampedCanFrame d_;
};    //end of class  Can_SCU_IPC_7_0x301

#endif    //end of __Can_SCU_IPC_7_0x301_H__
