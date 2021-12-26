#ifndef icvMsgpackRecorder_h
#define icvMsgpackRecorder_h

#include <fstream>
#include <map>
#include <chrono>
#include <boost/filesystem/path.hpp>
#include <msgpack.hpp>

#include <sstream>
#include <utility>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

using namespace std;
using namespace msgpack;
using namespace boost::interprocess;

#define icv_ratio  ::std::ratio
#define icv_chrono ::std::chrono

typedef icv_chrono::duration<std::int32_t, icv_ratio<1>> Duration32;
typedef icv_chrono::duration<std::int32_t, icv_ratio<1,1000>> Duration32ms;
typedef icv_chrono::duration<std::int64_t, icv_ratio<1,1000000>> Duration64us;
typedef icv_chrono::duration<std::int64_t, icv_ratio<1, 1000000000>> Duration64;
typedef icv_chrono::system_clock DefaultClock;
typedef icv_chrono::time_point<DefaultClock, Duration32> Time32;
typedef icv_chrono::time_point<DefaultClock, Duration64> Time64;
    
#define ICV_PROPERTY_GET(name, variable, type)          \
    type Get##name() { return variable; }               \
    const type Get##name() const { return variable; }   \

#define ICV_PROPERTY_GETSET(name, variable, type)       \
    ICV_PROPERTY_GET(name, variable, type)              \
    void Set##name(const type& name) { variable = name; }

#define ICV_PROPERTY_GETSET_PTR(name, variable, ptr)    \
    ICV_PROPERTY_GET(name, variable, ptr)               \
    void Set##name(ptr name) { variable = name; }

#include <boost/log/trivial.hpp>
#define ICV_LOG_TRACE BOOST_LOG_TRIVIAL(trace)
#define ICV_LOG_DEBUG BOOST_LOG_TRIVIAL(debug)
#define ICV_LOG_INFO  BOOST_LOG_TRIVIAL(info)
#define ICV_LOG_WARN  BOOST_LOG_TRIVIAL(warning)
#define ICV_LOG_ERROR BOOST_LOG_TRIVIAL(error)
#define ICV_LOG_FATAL BOOST_LOG_TRIVIAL(fatal)

    using namespace std;

    class icvObject {};

    static bool _synced = false;
    static Duration64 _offset = Duration64(0);
    
    struct SyncClock
    {
        typedef icv_chrono::system_clock base_clock;
        typedef Duration64 duration;
        //typedef Duration64::rep rep;
        //typedef Duration64::period period;
        typedef icv_chrono::time_point<SyncClock> time_point;
        typedef icv_chrono::time_point<SyncClock,Duration32> time_point_s;
        typedef icv_chrono::time_point<SyncClock,Duration32ms> time_point_ms;
        typedef icv_chrono::time_point<SyncClock,Duration64us> time_point_us;
        
        
        static bool is_steady() { return false; }
        static bool is_synced() { 
            return _synced;
        }

        static void sync(const duration& offset) {
            _offset = offset;
            _synced = true;
        }

        static time_point now() { 
            return time_point(base_clock::now().time_since_epoch() + _offset); 
        };
        
        static time_point_s now_s()
        {
            Duration64 dura_64=base_clock::now().time_since_epoch() + _offset;
            // Time32 time_32=icv_chrono::time_point_cast<Duration32>(time_64);
            //Duration64 asd=_offset+base_clock::now().time_since_epoch();
            time_point time_64 = time_point(dura_64);
            time_point_s time_32 = icv_chrono::time_point_cast<Duration32>(time_64);
            return time_32;
        };

	    static time_point_ms now_ms()
        {
            Duration64 dura_64=base_clock::now().time_since_epoch() + _offset;
            //Time32 time_32=icv_chrono::time_point_cast<Duration32>(time_64);
            //Duration64 asd=_offset+base_clock::now().time_since_epoch();
            time_point time_64=time_point(dura_64);
            time_point_ms time_32ms=icv_chrono::time_point_cast<Duration32ms>(time_64);
            return time_32ms;            
        };

	    static time_point_us now_us() {
            Duration64 dura_64=base_clock::now().time_since_epoch() + _offset;
            // Time32 time_32=icv_chrono::time_point_cast<Duration32>(time_64);
            //Duration64 asd=_offset+base_clock::now().time_since_epoch();
            time_point time_64=time_point(dura_64);
            time_point_us time_64us=icv_chrono::time_point_cast<Duration64us>(time_64);
            return time_64us;
        };

	    static time_point now_ns() 
        {
            Duration64 dura_64=base_clock::now().time_since_epoch() + _offset;
            // Time32 time_32=icv_chrono::time_point_cast<Duration32>(time_64);
            //Duration64 asd=_offset+base_clock::now().time_since_epoch();
            time_point time_64=time_point(dura_64);
            //time_point time_32=icv_chrono::time_point_cast<Duration32>(time_64);
            return time_64;            
        };

        static time_t time_s()   { return now_s().time_since_epoch().count();   }
	    static time_t time_ms()  { return now_ms().time_since_epoch().count(); }
	    static time_t time_us()  { return now_us().time_since_epoch().count(); }
	    static time_t time_ns()  { return now_ns().time_since_epoch().count(); };
        static const duration& offset() { return _offset; }
    };

    class icvSemaphore 
    {
    public:
        explicit icvSemaphore(int count=1) : count_(count),threadnum_(count) {

        }

        void Lock() {
            boost::unique_lock<boost::mutex> lock(mutex_);
            if(count_==0) condition_.wait(lock);
            if(count_>0)--count_;
        };
        
        void Lock(int ms) {
            assert(ms>0);
            boost::unique_lock<boost::mutex> lock(mutex_);
            if(!(count_>0))
        
            {
                boost::cv_status stat= condition_.wait_for(lock,boost::chrono::milliseconds(ms));
                if(stat==boost::cv_status::timeout)
                {
                    count_=threadnum_;
                    condition_.notify_all();

                }

            }
        };
        void Release(){
            boost::unique_lock<boost::mutex> lock(mutex_);

            if(count_<threadnum_)++count_;
            condition_.notify_one();
        };

        void Release_all(){
            boost::unique_lock<boost::mutex> lock(mutex_);

            count_=threadnum_;
            condition_.notify_all();
        } ;

    private:
        int count_;
        int threadnum_;
        boost::mutex mutex_;
        boost::condition_variable condition_;
    };    

    class icvDataObject : public icvObject, boost::noncopyable
    {
    public:
        virtual void Reserve() = 0; // If the data is already initialized, then this method just return
        virtual void Dispose() = 0; // If the data is already disposed, then this method just return
        bool Is_Reserved(){return is_reserved;};
        virtual uint64_t GetActualMemorySize() = 0;

        ICV_PROPERTY_GETSET(SourceTime, _sourceTime, time_t)

        // TODO: serialize timestamp separately
        virtual void Serialize(std::ostream& out, const uint32_t& version) const = 0;
        virtual void Deserialize(std::istream& in, const uint32_t& version) = 0;

        // TODO: Split initialization and data copy to help derived class implement this
        // TODO: Return shared pointer instead?
        virtual icvDataObject* DeepCopy() = 0;
        // ShallowCopy is not allowed for data object, please copy the pointer to data object directly
        // icvDataObject* ShallowCopy() = delete;
        
        virtual void CopyTo(icvDataObject* dst) = 0;

        virtual std::string Print() = 0; // For information display

        // Downcast this data object to T
        template <typename T> inline T& As() { return *(static_cast<T*>(this)); }
        template <typename T> inline const T& As() const { return *(static_cast<T*>(this)); }

    protected:
        time_t _sourceTime=0;
        bool is_reserved=false;
    };


        class icvCvMatData :public icvDataObject
        {
        public:
            typedef cv::Mat UType;
            typedef icvCvMatData SelfType;
            typedef struct ImageRecStruct
            {			
                string imagename;
                MSGPACK_DEFINE(imagename);	
            } ImageRecStruct;
        public:
        icvCvMatData(int rows, int cols, int type) :
            _init_type(type), _init_sizes {rows, cols} {

            Reserve();
        }

        icvCvMatData() :
            _init_type(CV_8UC3), _init_sizes {400, 600} {
            Reserve();
        }

        icvCvMatData(const vector<int>& sizes, int type)
            : _init_sizes(sizes), _init_type(type) {  Reserve();}
 
        virtual void Reserve() override
        {
            if (!_data) {
                _data = new cv::Mat(_init_sizes, _init_type);
                is_reserved = true;
            }
        }

        UType& getPic()
        {
            return *_data;
        }

        virtual void Dispose() override
        {
            if (_data) delete _data;
            _data = nullptr;
        }

        virtual uint64_t GetActualMemorySize() override
        {
            if (_data)
            {
                if (_data->isContinuous()) 
                    return _data->total() * _data->elemSize();
                else 
                    return _data->step[0] * _init_sizes[0];
            }
            else 
                return 0;
        }

        virtual void Serialize(std::ostream& out, const uint32_t& version) const override
        {
            std::stringstream buff ;
            pack(buff, _data_rec);

            // out <<this->GetSourceTime()<<" "<<buff.size()<<" "<< buff.data();
            out <<this->GetSourceTime()<<" "<<buff.str().size()<<" " << buff.str().data();
        };
     
        virtual void Deserialize(std::istream& in, const uint32_t& version) override
        {
            unpacked result;
            size_t len;
            string buff;
            time_t timestamp;

            in >> timestamp>>len >> buff;   
            this->SetSourceTime(timestamp);

            if (len == 0) 
                _data = nullptr;
            else 
            {
                // ICV_LOG_INFO<<"TIME length:"<<len;
                // ICV_LOG_INFO<<"TIME STAMP:"<<timestamp;
                unpack(&result, buff.c_str(), len);

                object obj = result.get();
                ImageRecStruct temp_V;
                obj.convert(temp_V);

                //ImageRecStruct _image_number=temp_V;
                //ICV_LOG_INFO<<"name STAMP:"<<temp_V.imagename;

                _data_rec.imagename=temp_V.imagename;
            }
        }

        string getImageName()
        {
            return _data_rec.imagename;
        }

        void SetImageName(const string& imagename) {
            _data_rec.imagename = imagename;
        }

        virtual icvDataObject* DeepCopy() override
        {
            icvCvMatData* copy = new icvCvMatData(_init_sizes, _init_type);

            copy->SetSourceTime (_sourceTime );
            //copy->setimagename(_sourceTime);

            copy->_init_sizes = _init_sizes;
            copy->_init_type = _init_type;

            copy->Reserve(); 
            // *(copy->_data)=_data->clone();
            copy->_data_rec.imagename=_data_rec.imagename;
            _data->copyTo(*(copy->_data));
            return copy;
        }

        virtual void CopyTo(icvDataObject* dst)  override 
        {
             SelfType* copy = static_cast< SelfType*>(dst);

            copy->SetSourceTime (_sourceTime );
            //copy->setimagename(_sourceTime);
            copy->_init_sizes = _init_sizes;
            copy->_init_type = _init_type;
            copy->_data_rec.imagename=_data_rec.imagename;
            copy->Reserve(); 

            *(copy->_data) = _data->clone();
        };

        void setoutvalue(const UType& data)
        {           
            _assign_shallow(data);
            time_t temp_time=SyncClock::time_us(); 
            _data_rec.imagename=to_string(temp_time);
            //ICV_LOG_INFO<<"SETOUT VALUE"<<temp_time;
            this->SetSourceTime(temp_time);
            //_image_name=to_string(temp_time);
        }

        icvCvMatData * get_Ptr()
        {
            return static_cast< icvCvMatData*> (this);
        }

        virtual std::string Print() override
        {
            string name = "cv::Mat shape:(";
            for (auto iter = _init_sizes.begin(); iter != _init_sizes.end(); iter++)
            {
                if (iter == _init_sizes.begin())
                    name.append(to_string(*iter));
                else 
                    name.append(", " + to_string(*iter));
            }
            name.append(")");
            return name;
        }

        operator cv::Mat&() { return *_data; }
        operator const cv::Mat&() const { return *_data; }

        UType* operator->() { return _data; }
        const UType* operator->() const { return _data; }

        icvCvMatData& operator = (const UType& data)
        {
            _assign(data);
            return *this;
        }

    private:
        cv::Mat * _data = nullptr;
        string _image_name;
        ImageRecStruct _data_rec ;
        // Params only for initialization
        std::vector<int> _init_sizes;
        int _init_type;
        vector<ImageRecStruct> _packdata;

    private:
      
        void _assign(const cv::Mat& data)
        {
            // Copy properties
            for (int i = 0; i < data.dims; i++)
                if (i < _init_sizes.size())
                    _init_sizes[i] = data.size[i];
                else
            _init_sizes.push_back(data.size[i]);
            _init_type = data.type();

            // Copy data
            *_data = data.clone();
        }

        void _assign_shallow(const cv::Mat& data)
        {
            // Copy properties
            for (int i = 0; i < data.dims; i++)
                if (i < _init_sizes.size())
                    _init_sizes[i] = data.size[i];
                else
                    _init_sizes.push_back(data.size[i]);
            _init_type = data.type();

            // Copy data
            *_data = data;
        }
    };

    template<typename T>
    class icvStructureData : public icvDataObject
    {

    public:
        typedef T              value_type;
        typedef T&             reference;
        typedef const T&       const_reference;

        icvStructureData() {
            Reserve();
        }

        virtual void Reserve() override 
        {
          
            if (!_data) 
            {
                   _data = new T;
            
                is_reserved=true;
          
            } 
        }
        virtual void Dispose() override { 
            if (_data) {
                delete _data; 
                _data = nullptr; 
            }
        }

        virtual uint64_t GetActualMemorySize() override 
        { 
            if (_data) 
                return sizeof(T);
            else 
                return 0;
        }

        // TODO: Not implemented yet
		virtual void Serialize(std::ostream& out, const uint32_t& version)const { 
            std::stringstream buff ;
            pack(buff, *_data);
            out <<this->GetSourceTime()<<" "<<buff.str().size()<<" " << buff.rdbuf();
        }
        
		virtual void Deserialize(std::istream& in, const uint32_t& version) { 
    
            // throw "Not implemented yet!"; 
            unpacked result;
            size_t len;
            string buff;
            time_t timestamp;
            ostringstream outstring;
            //in>>outstring.rdbuf();

            in >> timestamp >> len;  
            in >> outstring.rdbuf();
            buff = outstring.str();
            this->SetSourceTime(timestamp);      
            if (len == 0) 
                _data = nullptr;
            else 
            {
                //unpack(&result, buff.c_str(), len);
                unpack(&result, buff.c_str(), len);
                object obj = result.get();
                T temp_data;
                // Imu temp_data;
                //temp_data =obj.as<T>();
                obj.convert(temp_data);
                _data= &temp_data;
            }
        }

        icvStructureData<T>* get_Ptr(){
           static_cast< icvStructureData<T>*> (this); 
        }

        virtual icvDataObject* DeepCopy() override
        {
            icvStructureData<T>* copy = new icvStructureData<T>;
            copy->SetSourceTime(_sourceTime) ;
            copy->Reserve(); *(copy->_data) = T(*_data);
            return copy;
        }

        virtual void CopyTo(icvDataObject* dst) override
        {
            icvStructureData<T>* temp=static_cast< icvStructureData<T>*>(dst);
            temp->SetSourceTime(_sourceTime);
            temp->Reserve(); *(temp->_data) = T(*_data);
        }

         virtual std::string Print() override
         {
             return "not implemented";
         }

        operator reference () { return *_data; }
        operator const_reference () const { return *_data; }

        T & getStructData () { return *_data; }

        T* operator->() { return _data; }
        const T* operator->() const { return _data; }

        icvStructureData<T>& operator = (const T& data)
        {
            *_data = data;
            //this->SetSourceTime(SyncClock::now_us().time_since_epoch().count());
            return *this;
        }

        void setoutvalue( T& data)
        { 
            // ICV_LOG_INFO<<"debug 5.1";
            Reserve();
            // ICV_LOG_INFO<<"debug 5.2";
           
            _data = &data;
            // ICV_LOG_INFO<<"debug 5.3";
          
            this->SetSourceTime(SyncClock::now_us().time_since_epoch().count());
        }

        void setoutvalue( T& data,const time_t timestamp)
        {
            Reserve();
            _data = &data;
            this->SetSourceTime(timestamp);
        }
        
        inline bool operator ==(const icvStructureData<T>& rhs) const
        {
            return *_data == *(rhs._data);
        }

        inline bool operator !=(const icvStructureData<T>& rhs) const
        {
            return *_data != *(rhs._data);
        }

        inline bool operator ==(const T& rhs) const
        {
            return *_data == rhs;
        }

        inline bool operator !=(const T& rhs) const
        {
            return *_data != rhs;
        }

        MSGPACK_DEFINE(_data);

    protected:
        T * _data = nullptr;
       vector<T> _packdata;
    };


    class icvBaseRecorder
    {
    public:
        icvBaseRecorder(uint32_t version) : _version(version) {}

        // void Play() { _tstart = Time64::clock::now(); }
        virtual void Record(const icvDataObject* data, const uint32_t& sourcePort) = 0;

    protected:
        Time64 _tstart;
        uint32_t _version;
    };

    typedef struct RecordHeader
    {
        int version;
        bool clock_synced;
        time_t clock_offset;
        time_t MaxTime;
        time_t MinTime;
        time_t datacount;

        MSGPACK_DEFINE_ARRAY(version, clock_synced, clock_offset,MaxTime,MinTime,datacount);
    } RecordHeader ;
    
    class icvMsgpackRecorder : public icvBaseRecorder
    {
    public:
        icvMsgpackRecorder(const boost::filesystem::path& filepath, uint32_t version)
            : _path(filepath), icvBaseRecorder(version)
        {
            // deserialize from file
            if (boost::filesystem::exists(filepath))
            {
                file_mapping fmap(filepath.string().c_str(), read_only);
                mapped_region region(fmap, read_only);

                msgpack::object_handle result;
                msgpack::unpack(result, (char*)region.get_address(), region.get_size());

                auto obj = result.get();

                auto header = obj.via.array.ptr[0].as<RecordHeader>();

                header_.MaxTime=header.MaxTime;
                header_.MinTime=header.MinTime;

                //if (header.clock_synced) SyncClock::sync(Duration64(header.clock_offset));

                obj.via.array.ptr[1].convert(_buffer);
            }
        }

        virtual ~icvMsgpackRecorder()
        {
            header_.MaxTime=temp_max;

            std::fstream output(_path_write.string(), ios::binary | ios::out);
            msgpack::pack(output, make_pair(header_, _buffer));
            output.close();
        }

        void setRecordpath(string filepath) {
            //ICV_LOG_INFO << "  path : "<<filepath; 
            boost::filesystem::path folderpath(filepath);
            time_t now_time = SyncClock::time_s();
            // struct tm *local;
            // local=localtime(&now_time);
            // stringstream ss;
            // ss<<local->tm_hour<<local->tm_min<<local->tm_sec;
            //string filename=ss.str();
            string filename = to_string(now_time)+".icvbag";
            _path_write = folderpath/filename;
	        
        }

        virtual void Record(const icvDataObject* data, const uint32_t& source) override
        {
            if(first_)
            {
                header_.MinTime=data->GetSourceTime();;
                first_=false;
                start_record = SyncClock::time_s();
            }  

            if (_buffer.find(source) == _buffer.end()) {
                _buffer[source] = std::vector<std::string>();
            }

            stringstream ss;
            data->Serialize(ss, _version);
            temp_max = data->GetSourceTime();
            _buffer[source].push_back(ss.str());
            header_.MaxTime = temp_max;
            //std::cout<<"max time: "<<temp_max<<std::endl;
            
            if ((SyncClock::time_s()-start_record) > 34)
            {
                start_record=SyncClock::time_s();
                output.open(_path_write.string(), ios::binary | ios::out);	
                msgpack::pack(output, make_pair(header_, _buffer));
                output.close();
                std::cout<<"write file done"<<std::endl;
            }

        }

        void Record(const icvDataObject* data, const uint32_t& source, const int64_t timestamp ) ;

        void closefile() {
            header_.MaxTime=temp_max;
            // ICV_LOG_INFO << " Recording path : "<<_path_write.string(); 
            output.open(_path_write.string(), ios::binary | ios::out);	
            msgpack::pack(output, make_pair(header_, _buffer));

            output.close();
        }

        RecordHeader getheader(){return header_;};
        
    private:
        boost::filesystem::path _path;
        boost::filesystem::path _path_write;

        // do not use unordered_map for msgpack compability
        std::map<unsigned int, std::vector<std::string>> _buffer;
        std::map<unsigned int, unsigned int> _bufferIdx;
        int id_;
        time_t start_record;
        RecordHeader header_;
        bool first_=true;
        time_t temp_max;
        std::fstream output;
    };

        template <typename PointT>
        class icvPointCloudData : public icvDataObject
        {
        public:
            typedef ::pcl::PointCloud<PointT> UType;
            typedef icvPointCloudData<PointT> SelfType;

        public:
            // Point value argument could be used after initialization
            icvPointCloudData(uint32_t count) : _init_width(count), _init_height(1) {}
            icvPointCloudData(uint32_t width, uint32_t height) : _init_width(width), _init_height(height) {}
            icvPointCloudData() : _init_width(100), _init_height(1) {Reserve();}

            virtual void Reserve() override
            {
                if(!_data) {_data = new UType(_init_width, _init_height);is_reserved=true;}
            }
            virtual void Dispose() override
            {
                if (_data) delete _data;
                _data = nullptr;
            }

            virtual uint64 GetActualMemorySize() override
            {
                if (_data) return _data->size() * sizeof(PointT);
                else return 0;
            }

            virtual void Serialize(std::ostream& out, const uint32_t& version) const override
            {
                //fprint("Directly serialize icvPointCloudData is not supported, please convert it to icvTableData");
            }
            virtual void Deserialize(std::istream& in, const uint32_t& version) override
            {
                //fprint("Directly serialize icvPointCloudData is not supported, please convert it from icvTableData");
            }

            virtual icvDataObject* DeepCopy() override
            {
                SelfType* copy = new SelfType(_init_width, _init_height);

                copy->_sourceTime = _sourceTime;
                copy->_init_width = _init_width;
                copy->_init_height = _init_height;
                copy->Reserve(); *(copy->_data) = *_data;
                return copy;
            }
         virtual void CopyTo(icvDataObject* dst)  override {

                SelfType* copy = static_cast< SelfType*>(dst);
                copy->_sourceTime = _sourceTime;
                copy->_init_width = _init_width;
                copy->_init_height = _init_height;
                copy->Reserve(); *(copy->_data) = *_data;



         };
         



            SelfType * get_Ptr()
        {

            return static_cast< SelfType*> (this);

        }
        void setoutvalue(const UType& data)
            {
                 _assign(data);
                this->SetSourceTime(SyncClock::now_us().time_since_epoch().count());


            }
            virtual std::string Print() override
            {
                return "pcl::PointCloud size:(" +
                    std::to_string(_init_width) + " x " + std::to_string(_init_height) + ")";
            }

            operator UType&() { return *_data; }
            operator const UType&() const { return *_data; }
           
            UType* operator->() { return _data; }
            const UType* operator->() const { return _data; }

            icvPointCloudData<PointT>& operator = (const UType& data)
            {
                _assign(data);
                return *this;
            }

        private:
            UType * _data = nullptr;

            // Params only for initialization
            uint32_t _init_width, _init_height;

        private:
            void _assign(const ::pcl::PointCloud<PointT>& data)
            {
                // Copy properties
                _init_width = data.width;
                _init_height = data.height;

                // Copy data
                *_data = data;
            }
            
        };



#endif // icvMsgpackRecorder_h
