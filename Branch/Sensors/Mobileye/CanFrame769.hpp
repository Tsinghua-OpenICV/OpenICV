

#ifndef __CanFrame769_h__
#define __CanFrame769_h__
#include "structureCanMobileye.h"
#include "CanDecoder.h"

typedef unsigned char uint8;
typedef float float32;
typedef double float64;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;


class CanFrame769 
{
public:
	CanFrame769(){};
	~CanFrame769(){};
	void decode()
	{
	uint16 temp; mDecodeToUI16(&temp,d_.frame.data,0,16);
    data_.LaneHeadingC1 = (double)((temp-32767.0)/1024.0);			
	};
	void * data() { return &data_; }; 
	int size() { return sizeof(LaneInfoPart2); };
	void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
	LaneInfoPart2 data_;
	TimestampedCanFrame d_;
};


#endif // __CanFrame769_h__
