


#ifndef __CanFrame739_h__
#define __CanFrame739_h__

#include "structureCanMobileye.h"
#include "CanDecoder.h"



class CanFrame739 
{
public:
	CanFrame739(){};
	~CanFrame739(){};
	void decode()
{
	uint8_t temp; mDecodeToUI8(&temp,d_.frame.data,0,8);
	data_.id = temp;
	uint16_t temp1; mDecodeToUI16(&temp1,d_.frame.data,8,12);
	data_.y = temp1*0.0625;
	int16_t temp2;mDecodeToI16(&temp2,d_.frame.data,24,10);
	data_.x = - temp2*0.0625;
	mDecodeToUI8(&temp,d_.frame.data,52,3);
	data_.type = temp;
	mDecodeToUI8(&temp,d_.frame.data,56,3);
	data_.status = temp;
};
	void * data() { return &data_; }; 
	int size() { return sizeof(StructMobileyeObstacle1); };
	void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
	StructMobileyeObstacle1 data_;
	TimestampedCanFrame d_;
};



#endif // __CanFrame731_h__
