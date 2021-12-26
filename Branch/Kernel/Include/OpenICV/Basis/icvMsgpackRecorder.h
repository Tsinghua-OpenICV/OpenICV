#ifndef icvMsgpackRecorder_h
#define icvMsgpackRecorder_h

#include <fstream>
#include <map>
#include <boost/filesystem/path.hpp>
#include <msgpack.hpp>

#include "OpenICV/Basis/icvBaseRecorder.h"

namespace icv { 
    using namespace std;

    class icvMsgpackRecorder : public icvBaseRecorder
    {
    public:
        icvMsgpackRecorder(const boost::filesystem::path& filepath, uint32_t version);
        ~icvMsgpackRecorder();
        void setRecordpath(string pathof);
        virtual void Record(const icvDataObject* data, const string& source) ICV_OVERRIDE;
         void Record(const icvDataObject* data, const Uint32& source,const Int64 timestamp ) ;
         void closefile() ;

        virtual void PlayNext(icvDataObject* data, const string& source) ICV_OVERRIDE;
        RecordHeader getheader(){return header_;};
        vector<std::string> GetBufferKey(){
            vector<std::string> temp;
            for(auto x:_buffer){
                 temp.push_back(x.first);
            }
            return temp;
        }
    private:
        boost::filesystem::path _path;
        boost::filesystem::path _path_write;

        // do not use unordered_map for msgpack compability
        std::map<string, std::vector<std::string>> _buffer;
        std::map<string, Uint32> _bufferIdx;
        int id_;
        time_t start_record;
        RecordHeader header_;
        bool first_=true;
        time_t temp_max;
        std::fstream output;
    };
}

#endif // icvMsgpackRecorder_h
