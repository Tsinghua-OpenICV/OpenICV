/******************************************************************************
 * @file CanHRW_2EA.hpp
 * @brief CAN  receive - High Resolution Wheel Speed
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-07
 *****************************************************************************/

#ifndef __CANHRW_2EA_H__
#define __CANHRW_2EA_H__

#include "structureCanXP.h"


class  CanHRW_2EA
{
public:

  CanHRW_2EA(){};
  ~CanHRW_2EA(){}; 
  void decode(){
  data_.HRW_TranCycle = 20;
  data_.FL_WhlSpd = 0; 
  data_.FR_WhlSpd = 0; 
  data_.RL_WhlSpd = 0; 
  data_.RR_WhlSpd = 0; 

   uint8 *pData = d_.frame.data;
   int16 temp,temp1,temp2,temp3;
   
  temp =0;
  temp1 =0;
  temp=(short)(pData[0]&0xff);
  temp1=(short)(pData[1]&0xff);
  temp=temp<<8;
  data_.FL_WhlSpd=temp+temp1;
   
  temp =0;
  temp1 =0;
  temp=(short)(pData[2]&0xff);
  temp1=(short)(pData[3]&0xff);
  temp=temp<<8;
  data_.FR_WhlSpd=temp+temp1;

  temp =0;
  temp1 =0;
  temp=(short)(pData[4]&0xff);
  temp1=(short)(pData[5]&0xff);
  temp=temp<<8;
  data_.RL_WhlSpd=temp+temp1;

  temp =0;
  temp1 =0;
  temp=(short)(pData[6]&0xff);
  temp1=(short)(pData[7]&0xff);
  temp=temp<<8;
  data_.RR_WhlSpd=temp+temp1;
  };    //end of decode()
  CanFrameHRW_2EA* data() {return &data_; }; 
  int size() { return sizeof(CanFrameHRW_2EA); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};

protected:

private:
  CanFrameHRW_2EA data_;
  TimestampedCanFrame d_;
};    //end of class  CanHRW_2EA

#endif    //end of __CANHRW_2EA_H__
