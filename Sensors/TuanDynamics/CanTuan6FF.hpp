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

#ifndef __CanTUAN6FF_h__
#define __CanTUAN6FF_h__

#include "OpenICV/structure/structureCanTuan.h"

class  CanTuan6FF

{
public:


  CanTuan6FF(){};
  ~CanTuan6FF(){}; 
  void decode(){
	  //	typedef struct
//{
// bool PicWarn; 
// bool AudioWarn; 
// bool SteeringToRight;
// float SteeringAngle;
// bool PLA_actif; 
// bool TorqueToRight; 
// float Torque; 
// bool HCA_aktif; 
// float Acceleration; 
// bool ACC_aktif; 
// 
//}GL1000Message;//ox6FF;
uint8 *pData = d_.frame.data;
int16 temp,temp1;
data_.PicWarn=false;
data_.AudioWarn=false; 
data_.SteeringToRight=false; 
data_.PLA_actif=false;  
data_.HCA_aktif=false; 
data_.ACC_aktif=false;
data_.TorqueToRight=false;

temp =0;
temp=(short)(pData[0]&0x20);
temp=temp>>5;
data_.SteeringToRight = temp;

temp =0;
temp=(short)(pData[0]&0x10);
temp=temp>>4;
data_.TorqueToRight = temp;

temp =0;
temp=(short)(pData[0]&0x04);
temp=temp>>2;
data_.PLA_actif = temp;

temp =0;
temp=(short)(pData[0]&0x02);
temp=temp>>1;
data_.HCA_aktif = temp;

temp =0;
temp=(short)(pData[0]&0x01);
data_.ACC_aktif = temp;

temp =0;
temp=(short)(pData[2]&0x40);
temp=temp>>6;
data_.AudioWarn = temp;

temp =0;
temp=(short)(pData[2]&0x80);
temp=temp>>7;
data_.PicWarn = temp;

temp=0;
temp1=0;
temp=(short)(pData[1]&0xff);
temp1=(short)(pData[2]&0x7);
temp1=temp1<<8;
temp1+=temp;
data_.Acceleration=temp1*0.005-7.22; 

temp=0;
temp1=0;
temp=(short)(pData[3]&0xff);
temp1=(short)(pData[4]&0x01);
temp1=temp1<<8;
temp1+=temp;
data_.Torque=temp1*0.01; 

temp=0;
temp1=0;
temp=(short)(pData[5]&0xff);
temp1=(short)(pData[6]&0x1f);
temp1=temp1<<8;
temp1+=temp;
data_.Torque=temp1*0.1; 


  //std::cout << "Vehicule state: " << (int)data_.vehiculdeState << std::endl;
	  
  };
  void * data() {return &data_; }; 
  int size() { return sizeof(GL1000Message); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
  GL1000Message data_;
TimestampedCanFrame d_;
};


#endif

