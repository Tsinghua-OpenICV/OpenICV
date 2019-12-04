/*********************************************************************
//  created:    2017/12/15
//
//  author:     Kun JIANG
//              Copyright Tsinghua University
// 
//  version:    $Id: $
//
//  purpose:    
*********************************************************************/

#ifndef __CanTuan3DC_h__
#define __CanTuan3DC_h__

#include "OpenICV/structure/structureCanTuan.h"



class CanTuan3DC
        
{
public:

  CanTuan3DC(){};
  ~CanTuan3DC(){}; 
  void decode(){
 data_.EngineSpeedState=0;
  data_.EngineSpeed=0;
  uint8 *pData = d_.frame.data;
  int16 temp,temp1;
  
  temp=0;
  temp=(short)(pData[1]&0x01);
  data_.EngineSpeedState=temp;
  
  temp=0;
  temp=(short)(pData[2]&0xFF);
  temp1=0;
  temp1=(short)(pData[3]&0xFF);
  temp1=temp1<<8;
  temp1+=temp;
  data_.EngineSpeed=temp1*0.25;
  };
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameGateway); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
  CanFrameGateway data_;
TimestampedCanFrame d_;
};


#endif

