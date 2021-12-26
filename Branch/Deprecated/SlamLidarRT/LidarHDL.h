#ifndef icvLidarHDL_h
#define icvLidarHDL_h

#include "OpenICV/Core/icvFunction.h"

namespace icv
{
    class icvRandomSource : public icvFunction
    {
    public:
        icvRandomSource(boost::shared_ptr<const icv::icvMetaData> info);
        icvRandomSource();

        // virtual void ConfigurateInput(icvSubscriber** inputPorts) override {}

        // virtual void ConfigurateOutput(icvPublisher** outputPorts) override;

        virtual int Execute(icvDataObject** inData, icvDataObject** outData) override;

        ICV_PROPERTY_GETSET(Interval, _interval, int)

    private:
        int _interval = 100;
    };
}

#endif // icvLidarHDL_h
