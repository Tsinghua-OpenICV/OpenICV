#ifndef icvRandomSource_h
#define icvRandomSource_h

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"

namespace icv { namespace function
{
    using namespace icv::core;

    class icvRandomSource : public core::icvFunction
    {
    public:
        icvRandomSource(icv_shared_ptr<const core::icvMetaData> info);
        icvRandomSource();

   

        virtual void Execute() ICV_OVERRIDE;

        ICV_PROPERTY_GETSET(Interval, _interval, int)

    private:
        int _interval = 100;
        icv::data::icvInt64Data *tempdata;
        // icvSubscriber* subs1;
        // icvPublisher* pubs1;
    };
}}

#endif // icvRandomSource_h
