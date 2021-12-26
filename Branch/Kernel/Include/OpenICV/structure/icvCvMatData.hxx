#ifndef icvCvMatData_h
#define icvCvMatData_h

#include <msgpack.hpp>
#include <msgpack/versioning.hpp>
#include <msgpack/adaptor/adaptor_base.hpp>
#include <msgpack/meta.hpp>
#include <msgpack/object.hpp>
#include "OpenICV/Core/icvDataObject.h"
#include <boost/static_assert.hpp>
#include <boost/type_traits/is_pod.hpp>
#include <msgpack.hpp>
#include <string>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>


namespace msgpack {

/// @cond
//MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {
MSGPACK_API_VERSION_NAMESPACE(v1){
/// @endcond

namespace adaptor {

static const std::string OPENCV_MAT_KEYS[] = {
    "rows",
    "cols",
    "type",
    "data"
};

template<>
struct pack<cv::Mat> {
    template <typename Stream>
    packer<Stream>& operator()(msgpack::packer<Stream>& o, cv::Mat const& v) const {
        const int nbytes = v.total() * v.elemSize();
        o.pack_map(4);
        o.pack(OPENCV_MAT_KEYS[0]);
        o.pack(v.rows);
        o.pack(OPENCV_MAT_KEYS[1]);
        o.pack(v.cols);
        o.pack(OPENCV_MAT_KEYS[2]);
        o.pack(v.type());
        o.pack(OPENCV_MAT_KEYS[3]);
        o.pack_bin(nbytes);
        o.pack_bin_body((const char*)v.data, nbytes);
        return o;
    }
};

template <>
struct convert<cv::Mat> 
{
    msgpack::object const& operator()(msgpack::object const& o, cv::Mat& v) const 
    {
        if(o.type != msgpack::type::MAP)
            throw msgpack::type_error();
        if(o.via.map.size != 4)
            throw msgpack::type_error();

        std::string keys[4];
        for (int key_index = 0 ; key_index < 4 ; key_index++)
        {
            o.via.map.ptr[key_index].key.convert(keys[key_index]);
            if (keys[key_index] != OPENCV_MAT_KEYS[key_index])
                throw msgpack::type_error();
        }

        if (o.via.map.ptr[3].val.type != msgpack::type::BIN)
            throw msgpack::type_error();

        int rows, cols, type;
        o.via.map.ptr[0].val.convert(rows);
        o.via.map.ptr[1].val.convert(cols);
        o.via.map.ptr[2].val.convert(type);

        v = cv::Mat(rows, cols, type);
        const uint32_t nbytes = v.total() * v.elemSize();

        if (o.via.map.ptr[3].val.via.bin.size != nbytes)
            throw msgpack::type_error();

        memcpy(v.data, o.via.map.ptr[3].val.via.bin.ptr, nbytes);
        return o;
    }
};

} // namespace adaptor

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

}  // namespace msgpack








//#include "cv_mat_msgpack.hxx"
using namespace msgpack;
using namespace std;
using namespace icv::core;

namespace icv
{
    namespace data
    {


          class icvCvMatData :public icv::core::icvDataObject
        {
        public:
            typedef cv::Mat UType;
            typedef icvCvMatData SelfType;
         struct ImageRecStruct
        {			
            string imagename;
            string another_name;
            MSGPACK_DEFINE(imagename,another_name);	
        } ;
        public:
        icvCvMatData(int rows, int cols, int type) :
            _init_type(type), _init_sizes {rows, cols} {

            Reserve();
            }

        icvCvMatData() :
         _init_type(CV_8UC3), _init_sizes {400, 600} {

           Reserve();
        }
        icvCvMatData(UType data_ini):_init_type(CV_8UC3), _init_sizes {400, 600}
            {
                Reserve();

                setvalueDeep(data_ini);

            }
        icvCvMatData(const vector<int>& sizes, int type)
            : _init_sizes(sizes), _init_type(type) {  Reserve();}
 

        virtual void Reserve() ICV_OVERRIDE
        {

            if (!_data) {_data = new cv::Mat(_init_sizes, _init_type);is_reserved=true;}
        }
       UType& getvalue()
        {
        //  return tempdata;
        return *_data;

        }
     
       virtual void Dispose() ICV_OVERRIDE
        {
            if (_data) delete _data;
            _data = ICV_NULLPTR;
        }

        virtual Uint64 GetActualMemorySize() ICV_OVERRIDE
        {
            if (_data)
            {
                if (_data->isContinuous()) return _data->total() * _data->elemSize();
                else return _data->step[0] * _init_sizes[0];
            }
            else return 0;
        }
            virtual void Serialize(std::stringstream& out, const uint32_t& version) const ICV_OVERRIDE
            {
                std::vector<uchar> data_encode;
                cv::imencode(".jpg", *_data, data_encode);
                std::string str_encode(data_encode.begin(), data_encode.end());
                

               // out <<this->GetSourceTime()<<" "<<buff.size()<<" "<< buff.data();
                out <<this->GetSourceTime()<<" "<<str_encode.size()<<" " << str_encode;
               // cout<<"size of one image: "<<out.str().size()<<endl;
             //   ICV_LOG_INFO<<"data:"<<_data_rec.imagename <<"RECORD LENGTH"<<buff.size();

            }
            virtual void Deserialize(std::stringstream& in, const uint32_t& version) ICV_OVERRIDE
            {

            
            size_t len;
            string buff;
            time_t timestamp;
            ostringstream outstring;
            in >> timestamp>>len >> outstring.rdbuf(); 
            //cout<<"size of in: "<<in.str().size()<<" timestamp: "<<timestamp<<" len: "<<len<<"buff: "<<outstring.str().size()<<endl;
  
            this->SetSourceTime(timestamp);
            if(len==0) _data=ICV_NULLPTR;
                else 
                {    
             std::string str_tmp = outstring.str();
			std::vector<uchar> data(str_tmp.begin(), str_tmp.end());
			//cout<<"size of received data: "<<data.size()<<endl;
            //img_decode=mFrame;
			// tempdata = cv::imdecode(data, CV_LOAD_IMAGE_COLOR);
            tempdata = cv::imdecode(data, cv::IMREAD_COLOR);
            _data=&tempdata;
                }
            }
            



        string getImageName()
        {
            return _data_rec.imagename;
        }

            virtual icv::core::icvDataObject* DeepCopy() ICV_OVERRIDE
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

       
         virtual void CopyTo(icvDataObject* dst)  ICV_OVERRIDE 
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
            void setvalue(const UType& data)
            {           

            _assign_shallow(data);
  
            time_t temp_time=icvTime::time_us(); 
            _data_rec.imagename=to_string(temp_time);
 //ICV_LOG_INFO<<"SETOUT VALUE"<<temp_time;
            this->SetSourceTime(temp_time);
            //_image_name=to_string(temp_time);


            }
         void setvalueDeep(const UType& data)
            {           

            _assign(data);

            time_t temp_time=icvTime::time_us(); 
            _data_rec.imagename=to_string(temp_time);
 //ICV_LOG_INFO<<"SETOUT VALUE"<<temp_time;
            this->SetSourceTime(temp_time);
            //_image_name=to_string(temp_time);


            }

                    icvCvMatData * get_Ptr()
        {

            return static_cast< icvCvMatData*> (this);

        }
            virtual std::string Print() ICV_OVERRIDE
        {
            string name = "cv::Mat shape:(";
            for (auto iter = _init_sizes.begin(); iter != _init_sizes.end(); iter++)
            {
                if (iter == _init_sizes.begin())
                    name.append(to_string(*iter));
                else name.append(", " + to_string(*iter));
            }
            name.append(")");
            return name;
        }


          //  operator cv::Mat&() { return *_data; }
          //  operator const cv::Mat&() const { return *_data; }

           // UType* operator->() { return _data; }
           // const UType* operator->() const { return _data; }

            icvCvMatData& operator = (const UType& data)
            {
               setvalueDeep(data);

                return *this;
            }

                private:
            cv::Mat * _data = nullptr;
            cv::Mat tempdata;
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
    }
}
#endif