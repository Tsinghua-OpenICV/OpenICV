#ifndef icvBaseRecorder_h
#define icvBaseRecorder_h

#include "OpenICV/Core/icvObject.h"
#include "OpenICV/Core/icvDataObject.h"

namespace icv { namespace _impl
{
    using namespace icv::core;

    class icvBaseRecorder : public icvObject
    {
    public:
        icvBaseRecorder(uint32_t version) : _version(version) {}

        void Play() { _tstart = Time64::clock::now(); }
        virtual void Record(const icvDataObject* data, const Uint32& sourcePort) = 0;
        virtual void PlayNext(icvDataObject* data, const Uint32& sourcePort) = 0;

    protected:
        Time64 _tstart;
        uint32_t _version;
    };
}}

#endif // icvBaseRecorder_h
