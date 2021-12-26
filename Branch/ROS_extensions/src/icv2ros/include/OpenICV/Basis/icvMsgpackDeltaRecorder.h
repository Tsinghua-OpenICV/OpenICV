#ifndef icvMsgpackDeltaRecorder_h
#define icvMsgpackDeltaRecorder_h

#include <fstream>
#include <map>
#include <boost/filesystem/path.hpp>
#include <msgpack.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "OpenICV/Basis/icvBaseRecorder.h"

namespace icv { 
    using namespace std;

    class icvMsgpackDeltaRecorder : public icvBaseRecorder
    {
    public:
        icvMsgpackDeltaRecorder(const boost::filesystem::path& filepath, uint32_t version);
        ~icvMsgpackDeltaRecorder();
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
        int count=0;
        time_t start_record;
        RecordHeader header_;
        bool first_=true;
        time_t temp_max;
        std::fstream output;
        const string line_end_flag_bits="icvbag";
    };
}

#endif // icvMsgpackRecorder_h
