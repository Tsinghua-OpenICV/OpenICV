#ifndef icvBaseRecorder_h
#define icvBaseRecorder_h

#include "OpenICV/Core/icvObject.h"
#include "OpenICV/Core/icvDataObject.h"

namespace icv { 
    using namespace icv::core;
            struct RecordHeader
    {
        Int32 version;
        bool clock_synced;
        time_t clock_offset;
        time_t MaxTime;
        time_t MinTime;
        time_t datacount;

        MSGPACK_DEFINE_ARRAY(version, clock_synced, clock_offset,MaxTime,MinTime,datacount);
    };
    class icvBaseRecorder : public icvObject
    {
    public:
        icvBaseRecorder(uint32_t version) : _version(version) {}

        void Play() { _tstart = Time64::clock::now(); }
        virtual void Record(const icvDataObject* data, const std::string& sourcePort) = 0;
        virtual void PlayNext(icvDataObject* data, const std::string& sourcePort) = 0;

    protected:
        Time64 _tstart;
        uint32_t _version;
    };
}

#endif // icvBaseRecorder_h
