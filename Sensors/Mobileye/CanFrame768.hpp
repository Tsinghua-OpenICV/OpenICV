

#ifndef __CanFrame768_h__
#define __CanFrame768_h__


#include "structureCanMobileye.h"
#include "CanDecoder.h"


class CanFrame768 
{
public:
	CanFrame768(){};
	~CanFrame768(){};
	void decode()
	{
	data_.LaneType=(unsigned int)(mobileyeDecodeCAN(d_.frame.data,0,0,4,0));
	data_.LaneQuality=(unsigned int)(mobileyeDecodeCAN(d_.frame.data,0,4,2,0));
	data_.LaneDegree=(unsigned int)(mobileyeDecodeCAN(d_.frame.data,0,6,2,0));
	data_.LanePositionC0=(double)(mobileyeDecodeCAN(d_.frame.data,0,8,16,1)/256.0);
	data_.LaneCurvatureC2=(double)((mobileyeDecodeCAN(d_.frame.data,0,24,16,0)-0x7FFF)/1024/1000);
	data_.LaneCurvatureDerivC3=(double)((mobileyeDecodeCAN(d_.frame.data,0,40,16,0)-0x7FFF)/(1<<28));
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
