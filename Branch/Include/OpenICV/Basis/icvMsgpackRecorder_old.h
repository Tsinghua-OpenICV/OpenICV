#ifndef icvMsgpackRecorder_h
#define icvMsgpackRecorder_h

#include <fstream>
#include <map>
#include <boost/filesystem/path.hpp>
#include <msgpack.hpp>
#include "OpenICV/Core/icvMacros.h"
#include "OpenICV/Basis/icvBaseRecorder_old.h"

namespace icv { namespace _impl
{
    using namespace std;
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
    class icvMsgpackRecorder_old : public icvBaseRecorder_old
    {
    public:
        icvMsgpackRecorder_old(const boost::filesystem::path& filepath, uint32_t version);
        ~icvMsgpackRecorder_old();
        void setRecordpath(string pathof);
        virtual void Record(const icvDataObject* data, const Uint32& source)  ;
        // void Record(const icvDataObject* data, const Uint32& source,const Int64 timestamp ) ;
         void closefile() ;

        virtual void PlayNext(icvDataObject* data, const Uint32& source)  ;
        RecordHeader getheader(){return header_;};
    private:
        boost::filesystem::path _path;
        boost::filesystem::path _path_write;

        // do not use unordered_map for msgpack compability
        std::map<Uint32, std::vector<std::string>> _buffer;
        std::map<Uint32, Uint32> _bufferIdx;
        int id_;
        time_t start_record;
        RecordHeader header_;
        bool first_=true;
        time_t temp_max;
        std::fstream output;
    };
}}

#endif // icvMsgpackRecorder_h
