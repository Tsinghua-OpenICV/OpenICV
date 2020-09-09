

#ifndef __CanFrame439_h__
#define __CanFrame439_h__
#include "structureCanMobileye.h"
#include "CanDecoder.h"



class CanFrame439 
{
public:
	CanFrame439(){};
	~CanFrame439(){};
	void decode(){
	data_.id = mobileyeDecodeCAN(d_.frame.data,0,0,8,0);
	data_.y = mobileyeDecodeCAN(d_.frame.data,0,8,12,0)*0.0625;
	data_.x = - mobileyeDecodeCAN(d_.frame.data,0,24,10,1)*0.0625;
	data_.type = static_cast<ObstacleType>(mobileyeDecodeCAN(d_.frame.data,0,52,3,0));
	data_.status = static_cast<ObstacleStatus>(mobileyeDecodeCAN(d_.frame.data,0,56,3,0));
}
;
	void * data() { return &data_; }; 
	int size() { return sizeof(StructMobileyeObstacle1); };
	void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
	StructMobileyeObstacle1 data_;
	TimestampedCanFrame d_;
};



#endif // __CanFrame439_h__
