

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
    data_.LaneValid = (LaneValidStatus)mobileyeDecodeCAN(d_.frame.data,0,0,2,0);
    data_.LaneCurvature = mobileyeDecodeCAN(d_.frame.data,0,8,16,1)*0.00000381;
    data_.LaneHeading = mobileyeDecodeCAN(d_.frame.data,0,24,12,1)*0.0005;
    data_.LaneOffset = mobileyeDecodeCAN(d_.frame.data,0,36,12,1)*0.02;
    data_.LaneConf = mobileyeDecodeCAN(d_.frame.data,0,48,2,1);
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
