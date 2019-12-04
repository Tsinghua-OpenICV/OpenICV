

#ifndef __CanFrame43A_h__
#define __CanFrame43A_h__

#include "structureCanMobileye.h"
#include "CanDecoder.h"




class CanFrame43A 
{
public:
	CanFrame43A(){};
	~CanFrame43A(){};
	void decode(){
	data_.width = mobileyeDecodeCAN(d_.frame.data,0,8,8,0)*0.05;
	data_.age = mobileyeDecodeCAN(d_.frame.data,0,16,8,0);
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


#endif // __CanFrame43A_h__
