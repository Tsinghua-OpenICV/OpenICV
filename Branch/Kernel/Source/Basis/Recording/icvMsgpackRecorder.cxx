#include "OpenICV/Basis/icvMsgpackRecorder.h"

#include <sstream>
#include <utility>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/filesystem/convenience.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace std;
using namespace boost::interprocess;
namespace fs = boost::filesystem;

namespace icv { 


    // TODO: serialize clock offset
    icvMsgpackRecorder::icvMsgpackRecorder(const boost::filesystem::path& filepath, uint32_t version)
        : _path(filepath), icvBaseRecorder(version)
    {
        // deserialize from file
        if (fs::exists(filepath))
        {
          file_mapping fmap(filepath.string().c_str(), read_only);
            mapped_region region(fmap, read_only);

            //msgpack::object_handle msg;
		    msgpack::unpacked msg;
            msgpack::unpack(msg, (char*)region.get_address(), region.get_size());

            auto obj = msg.get();

            auto header = obj.via.array.ptr[0].as<RecordHeader>();
            
            header_.MaxTime=header.MaxTime;
            header_.MinTime=header.MinTime;

            //if (header.clock_synced) icvTime::sync(Duration64(header.clock_offset));

            obj.via.array.ptr[1].convert(_buffer);
           
           
        }
    }

    icvMsgpackRecorder::~icvMsgpackRecorder()
    {
        // save to file
        //RecordHeader header;
        //header.clock_synced = icvTime::is_synced();
        //header.clock_offset = icvTime::offset().count();
        header_.MaxTime=temp_max;

        //std::fstream output(_path_write.string(), ios::binary | ios::out);
        msgpack::pack(output, make_pair(header_, _buffer));
        output.close();
    }
    void icvMsgpackRecorder::closefile()
    {
        // save to file
        //RecordHeader header;
        //header.clock_synced = icvTime::is_synced();
        //header.clock_offset = icvTime::offset().count();
        header_.MaxTime=temp_max;
       // ICV_LOG_INFO << " Recording path : "<<_path_write.string(); 
         //output.open(_path_write.string(), ios::binary | ios::out);

        msgpack::pack(output, make_pair(header_, _buffer));

        output.close();
    }

    void icvMsgpackRecorder::setRecordpath(string filepath){
                ICV_LOG_INFO << "  path : "<<filepath; 

        boost::filesystem::path folderpath(filepath);
        time_t now_time=icvTime::time_s();
        // struct tm *local;
        // local=localtime(&now_time);
        // stringstream ss;
        // ss<<local->tm_hour<<local->tm_min<<local->tm_sec;

        //string filename=ss.str();
        string filename=to_string(now_time)+".icvbag";
        _path_write=folderpath/filename;
	output.open(_path_write.string(), ios::binary | ios::out);
    }

    void icvMsgpackRecorder::Record(const icvDataObject* data, const string& source)
    {
       
        if(first_)
        {
            header_.MinTime=data->GetSourceTime();;
            //cout<<"the minimum time is "<<header_.MinTime<<endl;
            first_=false;
            start_record=icvTime::time_s();
        }  
        if (_buffer.find(source) == _buffer.end())
            _buffer[source] = std::vector<std::string>();
        // mapStudent.insert(pair<int, string>(1, "student_one"));  
        stringstream ss;
        data->Serialize(ss, _version);
        temp_max=data->GetSourceTime();
        //cout<<"The content has the length of: "<<ss.str().length()<<endl;
        _buffer[source].push_back(ss.str());
        header_.MaxTime=temp_max;
        //cout<<"the maximum time is "<<header_.MaxTime<<endl;
       
        if((icvTime::time_s()-start_record)>2)
        {
            start_record=icvTime::time_s();
            output.open(_path_write.string(), ios::binary | ios::out);
    
            msgpack::pack(output, make_pair(header_, _buffer));
            output.close();
        }

    }

    void icvMsgpackRecorder::PlayNext(icvDataObject* data, const string& source)
    {
        // TODO: add delay into recording and playback
        if (_buffer.find(source) == _buffer.end()){
          ICV_THROW_MESSAGE("Cannot find data records for source");}
        if (_bufferIdx.find(source) == _bufferIdx.end()){
            _bufferIdx[source]=0;}
        
      
        stringstream ss(_buffer[source][_bufferIdx[source]]);
       
        //ICV_LOG_INFO<<"READ FROM FILE LIST "<<source<<":  "<< _buffer[source][_bufferIdx[source]];
        //cout<<_buffer[source].size()<<" "<<_bufferIdx[source]<<" "<<endl;
        if((_buffer[source].size()-_bufferIdx[source])>1) {
        _bufferIdx[source] ++;}
   
        data->Deserialize(ss, _version);
        data->empty=false;

    }
   

}
