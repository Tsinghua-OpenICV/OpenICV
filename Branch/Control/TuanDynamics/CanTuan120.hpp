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

#ifndef __CanTUAN120_h__
#define __CanTUAN120_h__


#include "OpenICV/structure/structureCanTuan.h"


class  CanTuan120
{
public:

  CanTuan120(){};
  ~CanTuan120(){}; 
  void decode(){



  data_.TSK_status=0;
  uint8 *pData = d_.frame.data;
  int16 temp;
  
  temp=0;
  temp=(short)(pData[3]&0x7);
  data_.TSK_status=temp;
  };
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameTouranTSK_06); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
  CanFrameTouranTSK_06 data_;
TimestampedCanFrame d_;
};



#endif

