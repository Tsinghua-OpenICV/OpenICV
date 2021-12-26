

#ifndef __CanFrame6FF_h__
#define __CanFrame6FF_h__

#include "structureCanMobileye.h"
#include "CanDecoder.h"



class CanFrame6FF 
{
public:
	CanFrame6FF(){};
	~CanFrame6FF(){};
	void decode(){
		uint8_t temp; mDecodeToUI8(&temp,d_.frame.data,0,8);
		data_.obstacleCount =temp ;
};
	void * data() { return &data_; }; 
	int size() { return sizeof(StructMobileyeObstaclesHeader); };
void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
	StructMobileyeObstaclesHeader data_;
	TimestampedCanFrame d_;
};


#endif // __CanFrame6FF_h__
