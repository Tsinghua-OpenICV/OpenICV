

#ifndef __CanFrame766_h__
#define __CanFrame766_h__

#include "structureCanMobileye.h"
#include "CanDecoder.h"




class CanFrame766 
{
public:
	CanFrame766(){};
	~CanFrame766(){};
	void decode()
	{


	uint8 temp1,temp2,temp3; int16 temp4; uint16 temp5,temp6;
	mDecodeToUI8(&temp1,d_.frame.data,0,4);
	mDecodeToUI8(&temp2,d_.frame.data,4,2);
	mDecodeToUI8(&temp3,d_.frame.data,6,2);
	mDecodeToI16(&temp4,d_.frame.data,8,16);
	mDecodeToUI16(&temp5,d_.frame.data,24,16);
	mDecodeToUI16(&temp6,d_.frame.data,40,16);


	data_.LaneType=temp1;
	data_.LaneQuality=temp2;
	data_.LaneDegree=temp3;
	data_.LanePositionC0=(double)(temp4/256.0);
	data_.LaneCurvatureC2=(double)((temp5-0x7FFF)/1024/1000);
	data_.LaneCurvatureDerivC3=(double)((temp6-0x7FFF)/(1<<28));
	};
	void * data() { return &data_; }; 
	int size() { return sizeof(LaneInfoPart1); };
	void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
	LaneInfoPart1 data_;
	TimestampedCanFrame d_;
};



#endif // __CanFrame766_h__
