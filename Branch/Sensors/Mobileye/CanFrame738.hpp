

#ifndef __CanFrame738_h__
#define __CanFrame738_h__

#include "structureCanMobileye.h"
#include "CanDecoder.h"




class CanFrame738 
{
public:
	CanFrame738(){};
	~CanFrame738(){};
	void decode()
{
	data_.obstacleCount = mobileyeDecodeCAN(d_.frame.data,0,0,8,0);
}
;
	void * data() { return &data_; }; 
	int size() { return sizeof(StructMobileyeObstaclesHeader); };
	void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
	StructMobileyeObstaclesHeader data_;
	TimestampedCanFrame d_;
};



#endif // __CanFrame738_h__
