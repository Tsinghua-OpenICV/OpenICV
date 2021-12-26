

#ifndef __CanFrame731_h__
#define __CanFrame731_h__


#include "structureCanMobileye.h"
#include "CanDecoder.h"



class CanFrame731 
{
public:
    CanFrame731()
    {
    }

    ~CanFrame731()
    {
    }

	void decode()
{
    uint8_t temp; mDecodeToUI8(&temp,d_.frame.data,0,2);
    data_.LaneValid =temp;
    int16_t temp1;mDecodeToI16(&temp1,d_.frame.data,8,16);
    data_.LaneCurvature = double(temp1*0.00000381);
    mDecodeToI16(&temp1,d_.frame.data,24,12);
    data_.LaneHeading =double(temp1*0.0005);
    mDecodeToI16(&temp1,d_.frame.data,36,12);
    data_.LaneOffset = double(temp1*0.02);
    char temp2; mDecodeToI8(&temp2,d_.frame.data,48,2);
    data_.LaneConf = temp2;
};

    void * data()
    {
        return &data_;
    }

    int size()
    {
        return sizeof(StructMobileyeLane);
    }
	void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};

protected:

private:
	StructMobileyeLane data_;
	TimestampedCanFrame d_;
};


#endif // __CanFrame731_h__
