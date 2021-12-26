/******************************************************************************
 * @file Can_SCU_IPC_5_0x208.hpp
 * @brief CAN  receive - SCU_IPC_5
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-07-13
 *****************************************************************************/

#ifndef __Can_SCU_IPC_5_0x208_H__
#define __Can_SCU_IPC_5_0x208_H__

#include "structureCanXP.h"

class  Can_SCU_IPC_5_0x208
{
public:

  Can_SCU_IPC_5_0x208(){};
  ~Can_SCU_IPC_5_0x208(){}; 
  void decode(){
  data_.DriverDoorLockSt = 0;
  data_.DriverDoorAjarSt = 0; 
  data_.PsngrDoorAjarSt = 0; 
  data_.RLDoorAjarSt = 0; 
  data_.RRDoorAjarSt = 0; 
  data_.LTurnLampOutputSt = 0; 
  data_.RTurnLampOutputSt = 0; 
  data_.HazardLampOutputSt = 0; 
  data_.LowBeamOutputSt = 0; 
  data_.HighBeamOutputSt = 0; 
  data_.Horndriverst = 0; 
  data_.FrontWiperOutputSt = 1; 
  data_.PowerMode = 0; 

  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;
  
  temp =0;
  temp=(short)(pData[0]&0x01);
  data_.DriverDoorLockSt=temp;
  
  temp =0;
  temp=(short)((pData[0]  >> 1) & 0x01);
  data_.DriverDoorAjarSt=temp;

  temp =0;
  temp=(short)((pData[0]  >> 2) & 0x01);
  data_.PsngrDoorAjarSt=temp;

  temp =0;
  temp=(short)((pData[0]  >> 3) & 0x01);
  data_.RLDoorAjarSt=temp;  

  temp =0;
  temp=(short)((pData[0]  >> 4) & 0x01);
  data_.RRDoorAjarSt=temp;

  temp =0;
  temp=(short)((pData[0]  >> 5) & 0x01);
  data_.LTurnLampOutputSt=temp;

  temp =0;
  temp=(short)((pData[0]  >> 6) & 0x01);
  data_.RTurnLampOutputSt=temp;

  temp =0;
  temp=(short)((pData[0]  >> 7) & 0x01);
  data_.HazardLampOutputSt=temp;

  temp =0;
  temp=(short)((pData[1]  >> 7) & 0x01);
  data_.LowBeamOutputSt=temp;

  temp =0;
  temp=(short)((pData[1]  >> 6) & 0x01);
  data_.HighBeamOutputSt=temp;

  temp =0;
  temp=(short)((pData[1]  >> 5) & 0x01);
  data_.Horndriverst=temp;

  temp =0;
  temp=(short)((pData[1]  >> 3) & 0x03);
  data_.FrontWiperOutputSt=temp;

  temp =0;
  temp=(short)((pData[1]  >> 1) & 0x03);
  data_.PowerMode=temp;
  };    //end of decode()
  CanFrame_SCU_IPC_5_0x208* data() {return &data_; }; 
  int size() { return sizeof(CanFrame_SCU_IPC_5_0x208); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:

private:
  CanFrame_SCU_IPC_5_0x208 data_;
  TimestampedCanFrame d_;
};    //end of class  Can_SCU_IPC_5_0x208

#endif    //end of __Can_SCU_IPC_5_0x208_H__
