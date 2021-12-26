

#ifndef __CanFrame63A_h__
#define __CanFrame63A_h__


#include "structureCanMobileye.h"
#include "CanDecoder.h"




class CanFrame63A 
{
public:
	CanFrame63A(){};
	~CanFrame63A(){};
	void decode()
{
		uint8_t temp; mDecodeToUI8(&temp,d_.frame.data,8,8);
		data_.width =double(temp*0.05) ;
		uint8_t temp1; mDecodeToUI8(&temp1,d_.frame.data,16,8);

	data_.age = temp1;
}
;
	void * data() { return &data_; }; 
	int size() { return sizeof(StructMobileyeObstacle2); };
	void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
	StructMobileyeObstacle2 data_;
	TimestampedCanFrame d_;
};



#endif // __CanFrame63A_h__
