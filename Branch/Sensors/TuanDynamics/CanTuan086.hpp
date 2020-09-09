

#ifndef __CanTUAN086_h__
#define __CanTUAN086_h__


#include "OpenICV/structure/structureCanTuan.h"




class  CanTuan086
{
public:

  CanTuan086(){};
  ~CanTuan086(){}; 
  void decode(){
   data_.LWI_Sensorstatus=0;
   data_.LWI_StrWhlAngleSt=0;
   data_.LWI_StrWhlAngleSize=0;
   data_.LWI_StrWhlAngleDrt=0;
   data_.LWI_StrWhlSpeedDrt=0;
   data_.LWI_StrWhlSpeedSize=0;
   
  uint8 *pData = d_.frame.data;
   int16 temp,temp1;
   
   temp =0; 
   temp=(short)(pData[1]&0x10);
   temp=temp>>4;
   data_.LWI_Sensorstatus=temp;
   
    temp =0; 
   temp=(short)(pData[1]&0x80);
   temp=temp>>7;
   data_.LWI_StrWhlAngleSt=temp;
   
   
   temp =0; 
   temp1=0;
   temp=(short)(pData[2]&0xFF);
   temp1=(short)(pData[3]&0x1F);
   temp1=temp1<<8;
   temp1+=temp;
   data_.LWI_StrWhlAngleSize=temp1*0.1;
   
   temp =0; 
   temp=(short)(pData[3]&0x20);
   temp=temp>>5;
   data_.LWI_StrWhlAngleDrt=temp;
   
   temp =0; 
   temp=(short)(pData[3]&0x40);
   temp=temp>>6;
   data_.LWI_StrWhlSpeedDrt=temp;
   
   temp =0; 
   temp1=0;
   temp=(short)(pData[3]&0x80);
   temp=temp>>7;
   temp1=(short)(pData[4]&0xFF);
   temp1=temp1<<1;
   temp1+=temp;
   data_.LWI_StrWhlSpeedSize=temp1*5;


  //std::cout << "Vehicule state: " << (int)data_.vehiculdeState << std::endl;
  };
  void * data() {return &data_; }; 
  int size() { return sizeof(CanFrameTuanLWI_01); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
  CanFrameTuanLWI_01 data_;
TimestampedCanFrame d_;
};



#endif

