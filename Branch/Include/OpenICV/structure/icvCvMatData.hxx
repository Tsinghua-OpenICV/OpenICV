#ifndef icvCvMatData_h
#define icvCvMatData_h

#include "OpenICV/Core/icvDataObject.h"
#include <boost/static_assert.hpp>
#include <boost/type_traits/is_pod.hpp>
#include <msgpack.hpp>
#include <string>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

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
                 std::stringstream buff ;
                pack(buff, _data_rec);


               // out <<this->GetSourceTime()<<" "<<buff.size()<<" "<< buff.data();
                out <<this->GetSourceTime()<<" "<<buff.str().size()<<" " << buff.str().data();

             //   ICV_LOG_INFO<<"data:"<<_data_rec.imagename <<"RECORD LENGTH"<<buff.size();

            }
            virtual void Deserialize(std::stringstream& in, const uint32_t& version) ICV_OVERRIDE
            {

            unpacked result;
            size_t len;
            string buff;
            time_t timestamp;

            in >> timestamp>>len >> buff;   
           

            this->SetSourceTime(timestamp);
        //    ICV_LOG_INFO<<"Dese STAMP:"<<timestamp;

            if(len==0) _data=ICV_NULLPTR;
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
            cv::Mat * _data = ICV_NULLPTR;
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