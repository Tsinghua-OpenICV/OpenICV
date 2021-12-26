#ifndef icvPythonData_hxx
#define icvPythonData_hxx

#include <boost/static_assert.hpp>
#include <boost/type_traits/is_pod.hpp>
//#include <msgpack.hpp>
#include <string>
#include "OpenICV/Core/icvDataObject.h"
#include "OpenICV/structure/openicv_ros.h"
using namespace msgpack;
using namespace std;

namespace icv { namespace data
{
    // TODO: add check for both to_string and msgpack serializtion implementation for custom data
    // TODO: add this to other PrimitiveData types (array, table, tensor)
    // TODO: use member function instead of to_string function. Besides, if T doesn't support print, then just disable printing precise information
    template<typename PyData>
    class icvPythonData : public icv::core::icvDataObject
    {

    public:
        typedef PyData&             reference;
        typedef const PyData&       const_reference;

        icvPythonData( PyData* data) {
            Reserve();
            _data = data; 
            this->SetSourceTime(icvTime::time_us() );
           //SetSerializeType(icvDataSerializeType::PythonStruct);
        }

        icvPythonData(PyData & data_ini) {
        setvalue(data_ini);
        //SetSerializeType(icvDataSerializeType::PythonStruct);

        }
        icvPythonData() {
        Reserve();
        //SetSerializeType(icvDataSerializeType::PythonStruct);

        }
        virtual void Reserve() ICV_OVERRIDE 
        {
          
            if (!_data) 
            {
                  _data = new PyData();
            
                is_reserved=true;
          
            } 
        }
        virtual void Dispose() ICV_OVERRIDE { if (_data) delete _data; _data = ICV_NULLPTR; }

        virtual Uint64 GetActualMemorySize() ICV_OVERRIDE 
        { 
            if (_data) return sizeof(PyData);
            else return 0;
        }

        // TODO: Not implemented yet


		virtual void Serialize(std::stringstream& out, const uint32_t& version) const ICV_OVERRIDE { 
           // std::stringstream buff ;
          //  pack(buff, *_data);
           // out <<this->GetSourceTime()<<" "<<buff.str().size()<<" " << buff.rdbuf();
            string buff_in;
            // ICV_LOG_INFO<<"DATA IS"<< this->_data.width;
            encode_python( this->_data,buff_in);
            ICV_LOG_INFO<<"SERIALIZE LENGTH1: "<<buff_in.length();
            out<<buff_in;
            //ICV_LOG_INFO<<"SERIALIZE LENGTH2: "<<buff_in.c_str().length();

         }
		virtual void Deserialize(std::stringstream& in, const uint32_t& version)  ICV_OVERRIDE{ 
    
          
            string buff;
            buff=in.str();
            decode_python(_data,buff);

            
        }
        void decode_python(PyData* data_python, string &message){
           // ICV_LOG_INFO<<"DECODE PYTHON";
           data_python->decode_from_python(message);

        }
        void encode_python(PyData* data_python, string &message)  const {
          
           data_python->encode_into_python(message);
           ICV_LOG_INFO<<"ENCODE PYTHON LENGTH: "<<message.length();
        }
       icvPythonData* get_Ptr(){static_cast< icvPythonData*> (this); }

        virtual core::icvDataObject* DeepCopy() ICV_OVERRIDE
        {
            icvPythonData* copy = new icvPythonData<PyData>();
            copy->SetSourceTime(_sourceTime) ;
            copy->Reserve(); *(copy->_data) = PyData(*_data);
            return copy;
        }
        virtual void CopyTo(core::icvDataObject* dst) ICV_OVERRIDE
        {

            icvPythonData* temp=static_cast< icvPythonData*>(dst);
            temp->SetSourceTime(_sourceTime);
            temp->Reserve(); 
            *(temp->_data) = PyData(*_data);


        }

         virtual std::string Print() ICV_OVERRIDE
         {
             return "not implemented";
         }

        operator reference () { return *_data; }
        operator const_reference () const { return *_data; }

        PyData & getvalue () { return *_data; }

        PyData* operator->() { return _data; }
        const PyData* operator->() const { return _data; }

        icvPythonData & operator = (const PyData& data)
        {
            *_data = data;
            return *this;
        }


        void setvalue( PyData& data)
        { 
         
            Reserve();          
            _data = &data;          
            this->SetSourceTime(icvTime::time_us() );
        }
        void setvaluePtr( PyData* data)
        { 

            Reserve();    
      
            _data = data; 
            this->SetSourceTime(icvTime::time_us() );

        }
        void setvalue( PyData& data,const time_t timestamp)
        {
            Reserve();
            _data = &data;
            this->SetSourceTime(timestamp);
        }
        
  

    protected:
        PyData * _data = ICV_NULLPTR;
       // PyData* _data_tem;
       //vector<PyData> _packdata;
       //T temp_data;

    };

    //#define _ICV_DECLARE_BASIC_DATA(type) typedef icvPythonData<type> icv##type##Data;
    //ICV_BASIC_TYPES_TEMPLATE(_ICV_DECLARE_BASIC_DATA);
   // #undef _ICV_DECLARE_BASIC_DATA

    // Common uses
    // typedef icvPythonData<IndexType> icvIndexData;
    // typedef icvPythonData<int>       icvIntData;
    // typedef icvPythonData<double>    icvDoubleData;
}}

#endif // icvPythonData_hxx
