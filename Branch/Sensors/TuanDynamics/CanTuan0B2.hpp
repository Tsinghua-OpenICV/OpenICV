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

#ifndef __CanTUAN0B2_h__
#define __CanTUAN0B2_h__


#include "OpenICV/structure/structureCanTuan.h"

#include <iostream>



class  CanTuan0B2
{
public:
  CanTuan0B2(){};
  ~CanTuan0B2(){}; 
  void decode()
  {
//	typedef  struct
//{
//
//	bool LeftFrontSpeed;         
//	bool RightFrontSpeed;
//	bool LeftRearSpeed;
//	bool RightRearSpeed;
//
//} CanFrameTuanESP_19;//0x0B2;



data_.LF_WhlSpd=0;
  data_.LR_WhlSpd=0;
  data_.RF_WhlSpd=0;
  data_.RR_WhlSpd=0;
  uint8 *pData = d_.frame.data;
  int16 temp,temp1;
  
  temp=0;
  temp=(short)(pData[4]&0xFF);
  temp1=0;
  temp1=(short)(pData[5]&0xFF);
  temp1=temp1<<8;
  temp1+=temp;
  data_.LF_WhlSpd=temp1*0.0075;
  
  temp=0;
  temp=(short)(pData[0]&0xFF);
  temp1=0;
  temp1=(short)(pData[1]&0xFF);
  temp1=temp1<<8;
  temp1+=temp;
  data_.LR_WhlSpd=temp1*0.0075;

  temp=0;
  temp=(short)(pData[6]&0xFF);
  temp1=0;
  temp1=(short)(pData[7]&0xFF);
  temp1=temp1<<8;
  temp1+=temp;
  data_.RF_WhlSpd=temp1*0.0075;

  temp=0;
  temp=(short)(pData[2]&0xFF);
  temp1=0;
  temp1=(short)(pData[3]&0xFF);
  temp1=temp1<<8;
  temp1+=temp;
  data_.RR_WhlSpd=temp1*0.0075;

  // // ROS_INFO("LF_WhlSpd:%f",data_.LF_WhlSpd);
  // // ROS_INFO("LR_WhlSpd:%f",data_.LR_WhlSpd);
  // // ROS_INFO("RF_WhlSpd:%f",data_.RF_WhlSpd);
  // // ROS_INFO("RR_WhlSpd:%f",data_.RR_WhlSpd);

};
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameTuanESP_19); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
private:
  CanFrameTuanESP_19 data_;
TimestampedCanFrame d_;
};



#endif

