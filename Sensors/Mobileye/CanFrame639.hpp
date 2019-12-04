

#ifndef __CanFrame639_h__
#define __CanFrame639_h__


#include "structureCanMobileye.h"
#include "CanDecoder.h"


class CanFrame639 
{
public:
	CanFrame639(){};
	~CanFrame639(){};
	void decode();
	void * data() { return &data_; }; 
	int size() { return sizeof(StructMobileyeObstacle1); };
	void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:
private:
	StructMobileyeObstacle1 data_;
	TimestampedCanFrame d_;
};



#endif // __CanFrame639_h__
