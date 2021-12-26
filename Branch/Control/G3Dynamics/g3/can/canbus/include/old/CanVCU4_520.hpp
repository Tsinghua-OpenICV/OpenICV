/******************************************************************************
 * @file CanVCU4_520.hpp
 * @brief CAN  receive - VCU_4
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-08
 *****************************************************************************/

#ifndef __CanVCU4_520_H__
#define __CanVCU4_520_H__

#include "structureCanXP.h"


class  CanVCU4_520
{
public:

  CanVCU4_520(){};
  ~CanVCU4_520(){}; 
  void decode(){
  data_.VCU4_TranCycle = 50;
  data_.VehicleSpeed = 0; 
  data_.GearsSt = 0; 
  data_.ReadySt = 0; 
  data_.BrakeSt = 0; 

  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;
  
  temp =0;
  temp1 =0;
  temp=(short)(pData[0]&0xff);
  temp1=(short)(pData[1]&0xff);
  temp=temp<<8;
  data_.VehicleSpeed=temp+temp1;

  temp =0;
  temp=(short)(pData[4]&0x03);
  data_.GearsSt=temp;
   
  temp =0;
  temp=(short)(pData[4]&0x04);
  temp=temp>>2;
  data_.ReadySt=temp;

  temp =0;
  temp=(short)(pData[7]&0x04);
  temp=temp>>2;
  data_.BrakeSt=temp;
  };    //end of decode()
  CanFrameVCU4_520* data() {return &data_; }; 
  int size() { return sizeof(CanFrameVCU4_520); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};

protected:

private:
  CanFrameVCU4_520 data_;
  TimestampedCanFrame d_;
};    //end of class  CanVCU4_520

#endif    //end of __CanVCU4_520_H__
