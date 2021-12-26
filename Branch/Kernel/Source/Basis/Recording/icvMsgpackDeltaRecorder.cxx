#include "OpenICV/Basis/icvMsgpackDeltaRecorder.h"

#include <sstream>
#include <utility>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/filesystem/convenience.hpp>

using namespace std;
using namespace boost::interprocess;
namespace fs = boost::filesystem;

namespace icv { 



    // TODO: serialize clock offset
    icvMsgpackDeltaRecorder::icvMsgpackDeltaRecorder(const boost::filesystem::path& filepath, uint32_t version)
        : _path(filepath), icvBaseRecorder(version)
    {
        // deserialize from file
        if (fs::exists(filepath))
        {

            file_mapping fmap(filepath.string().c_str(), read_only);
            mapped_region region(fmap, read_only);

            stringstream readbuf_all;
            readbuf_all.rdbuf()->sputn((char*)region.get_address(),region.get_size());
            //cout<<"file size:"<<region.get_size()<<" readbuf_all: "<<readbuf_all.str()<<endl;

            size_t length_; string line_single;
            vector <std::map<string, std::vector<std::string>>> _buffer_vector;
            int id_ = 0;
            //readbuf_all>>length_;
            std::string line_all=readbuf_all.str();
            vector <string> fields;
            boost::split_regex( fields, line_all, boost::regex( line_end_flag_bits ) );
            for (int i=0;i<fields.size();i++){
                //cout<<"each length is: "<<fields[i].size()<<endl;
                if(fields[i].size()>0){
                stringstream temps;
                temps.str(fields[i]);
                temps>>length_;
                ostringstream outstring;
                temps>>outstring.rdbuf();
                cout<<"length of readbuf_all: "<<length_<<" line_single: "<<outstring.str().size()<<" temps:"<<temps.str().size()<<endl;
                msgpack::object_handle result;
                msgpack::unpack(result, (char*)outstring.str().c_str(), length_);

                auto obj = result.get();

                auto header = obj.via.array.ptr[0].as<RecordHeader>();
                cout<<"max time: "<<header.MaxTime<<" min time: "<<header.MinTime<<endl;
                if(i==0){header_.MinTime = header.MinTime;}
                header_.MaxTime = max(header.MaxTime, header_.MaxTime);

                    std::map<string, std::vector<std::string>> buff_single;
                    obj.via.array.ptr[1].convert(buff_single);
                    _buffer_vector.push_back(buff_single);
                }
            }

            // while (getline(readbuf_all,line_single,line_end_flag_bits))
            // {
                
            //     id_++;
            //     stringstream temps;
            //     temps.str(line_single);
            //     temps>>length_;
            //     ostringstream outstring;
            //     temps>>outstring.rdbuf();
            //     cout<<"length of readbuf_all: "<<length_<<" line_single: "<<outstring.str().size()<<" temps:"<<temps.str().size()<<endl;
            //     msgpack::object_handle result;
            //     msgpack::unpack(result, (char*)outstring.str().c_str(), length_);
                
            //     auto obj = result.get();

            //     auto header = obj.via.array.ptr[0].as<RecordHeader>();

            //     if (id_ == 1) header_.MinTime = header.MinTime;
            //     header_.MaxTime = header.MaxTime;

            //         std::map<string, std::vector<std::string>> buff_single;
            //         obj.via.array.ptr[1].convert(buff_single);
            //         _buffer_vector.push_back(buff_single);
                
            // }
            _buffer.insert(_buffer_vector[0].begin(), _buffer_vector[0].end());
            std::map<string, std::vector<std::string>>::iterator iter_;
            for (int i = 1; i < _buffer_vector.size(); i++)
            {
                for (iter_ = _buffer_vector[i].begin(); iter_ != _buffer_vector[i].end(); ++iter_)
                {

                    if (!_buffer.insert(std::pair<string, std::vector<std::string>>(iter_->first, iter_->second)).second)
                    {
                        _buffer[iter_->first].insert(_buffer[iter_->first].end(), iter_->second.begin(), iter_->second.end());

                    }


                }







            }


      

            //if (header.clock_synced) icvTime::sync(Duration64(header.clock_offset));

        }
    }

    icvMsgpackDeltaRecorder::~icvMsgpackDeltaRecorder()
    {
        // save to file
        //RecordHeader header;
        //header.clock_synced = icvTime::is_synced();
        //header.clock_offset = icvTime::offset().count();
        header_.MaxTime=temp_max;

        //std::fstream output(_path_write.string(), ios::binary | ios::out);
        //msgpack::pack(output, make_pair(header_, _buffer));
        output.close();
    }
    void icvMsgpackDeltaRecorder::closefile()
    {
        // save to file
        //RecordHeader header;
        //header.clock_synced = icvTime::is_synced();
        //header.clock_offset = icvTime::offset().count();
        header_.MaxTime=temp_max;
       // ICV_LOG_INFO << " Recording path : "<<_path_write.string(); 
         //output.open(_path_write.string(), ios::binary | ios::out);

        //msgpack::pack(output, make_pair(header_, _buffer));

        output.close();
    }

    void icvMsgpackDeltaRecorder::setRecordpath(string filepath){
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

    void icvMsgpackDeltaRecorder::Record(const icvDataObject* data, const string& source)
    {
        std::stringstream stream_rec;

        if(first_)
        {
            header_.MinTime=data->GetSourceTime();
            cout<<"the minimum time of the bag is: "<<header_.MinTime<<endl;
            first_=false;
            start_record=icvTime::time_s();
        }  
        if (_buffer.find(source) == _buffer.end())
            _buffer[source] = std::vector<std::string>();

        stringstream ss;
        data->Serialize(ss, _version);
        temp_max=data->GetSourceTime();
        _buffer[source].push_back(ss.str());
        header_.MaxTime=temp_max;
        count++;
        cout<<"recorded how many points: "<<count<<endl;
        if((icvTime::time_s()-start_record)>0.5)
        {
            start_record=icvTime::time_s();
            msgpack::pack(stream_rec, make_pair(header_, _buffer));
            output.open(_path_write.string(), std::fstream::in|std::fstream::out|std::fstream::app);
			//output.open(_path_write.string(), ios::binary | ios::out);
            cout<<"length of writebuf_all: "<<stream_rec.str().size()<<endl;
            first_=true;
            output << stream_rec.str().size() << " " << stream_rec.str()<<line_end_flag_bits;
            _buffer.clear();
            output.close();
        }

    }

    void icvMsgpackDeltaRecorder::PlayNext(icvDataObject* data, const string& source)
    {
        // TODO: add delay into recording and playback
        // if (_buffer.find(source) == _buffer.end())
        //  ICV_THROW_MESSAGE("Cannot find data records for source");

        if (_bufferIdx.find(source) == _bufferIdx.end())
            _bufferIdx[source]=0;
        stringstream ss(_buffer[source][_bufferIdx[source]]);
        // ICV_LOG_INFO<<"READ FROM FILE LIST "<<source<<":  "<< _buffer[source][_bufferIdx[source]].length();
        if((_buffer[source].size()-_bufferIdx[source])>1) _bufferIdx[source] ++;
        data->Deserialize(ss, _version);

    }
}
