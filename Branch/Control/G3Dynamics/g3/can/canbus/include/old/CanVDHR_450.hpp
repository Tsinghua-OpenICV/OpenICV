/******************************************************************************
 * @file CanVDHR_450.hpp
 * @brief CAN  receive - VDHR
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-08
 *****************************************************************************/

#ifndef __CanVDHR_450_H__
#define __CanVDHR_450_H__

#include "structureCanXP.h"


class  CanVDHR_450
{
public:

  CanVDHR_450(){};
  ~CanVDHR_450(){}; 
  void decode(){
  data_.VDHR_TranCycle = 1000;
  data_.TotalVehicleDistance = 0; 

  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;
  
  temp =0;
  temp1 =0;
  temp2 = 0;
  temp3 = 0;
  temp=(short)(pData[0]&0xff);
  temp1=(short)(pData[1]&0xff);
  temp2=(short)(pData[2]&0xff);
  temp3=(short)(pData[3]&0xff);
  temp=temp<<24;
  temp1=temp1<<16;
  temp2=temp2<<8;
  data_.TotalVehicleDistance=temp + temp1 + temp2 + temp3;
  };    //end of decode()
  CanFrameVDHR_450* data() {return &data_; }; 
  int size() { return sizeof(CanFrameVDHR_450); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};

protected:

private:
  CanFrameVDHR_450 data_;
  TimestampedCanFrame d_;
};    //end of class  CanVDHR_450

#endif    //end of __CanVDHR_450_H__
