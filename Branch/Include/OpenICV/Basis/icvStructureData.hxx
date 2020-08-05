#ifndef icvStructureData_hxx
#define icvStructureData_hxx

#include <boost/static_assert.hpp>
#include <boost/type_traits/is_pod.hpp>
#include <msgpack.hpp>
#include <string>
#include "OpenICV/Core/icvDataObject.h"
using namespace msgpack;
using namespace std;

namespace icv { namespace data
{
    // TODO: add check for both to_string and msgpack serializtion implementation for custom data
    // TODO: add this to other PrimitiveData types (array, table, tensor)
    // TODO: use member function instead of to_string function. Besides, if T doesn't support print, then just disable printing precise information
   
    template<typename T>
    class icvStructureData : public icv::core::icvDataObject
    {

    public:
        typedef T              value_type;
        typedef T&             reference;
        typedef const T&       const_reference;

        icvStructureData() {
            Reserve();
        }

        icvStructureData(value_type & data_ini) {
        setvalue();
        }
        virtual void Reserve() ICV_OVERRIDE 
        {
          
            if (!_data) 
            {
                   _data = new T;
            
                is_reserved=true;
          
            } 
        }
        virtual void Dispose() ICV_OVERRIDE { if (_data) delete _data; _data = ICV_NULLPTR; }

        virtual Uint64 GetActualMemorySize() ICV_OVERRIDE 
        { 
            if (_data) return sizeof(T);
            else return 0;
        }

        // TODO: Not implemented yet


		virtual void Serialize(std::stringstream& out, const uint32_t& version)const { 
            std::stringstream buff ;
            pack(buff, *_data);
            out <<this->GetSourceTime()<<" "<<buff.str().size()<<" " << buff.rdbuf();
         }
		virtual void Deserialize(std::stringstream& in, const uint32_t& version) { 
    
            // throw "Not implemented yet!"; 
            unpacked result;
            size_t len;
            string buff;
            time_t timestamp;
            ostringstream outstring;

            in >> timestamp >> len;  
            in >> outstring.rdbuf();
            buff=outstring.str();
            this->SetSourceTime(timestamp);      
            if(len==0) _data=ICV_NULLPTR;
            else 
            {
                unpack(&result, buff.c_str(), len);
                object obj = result.get();
                 T temp_data; 
                obj.convert(temp_data);
                _data= &temp_data;
            }
            
            
        }

       icvStructureData<T>* get_Ptr(){static_cast< icvStructureData<T>*> (this); }

        virtual core::icvDataObject* DeepCopy() ICV_OVERRIDE
        {
            icvStructureData<T>* copy = new icvStructureData<T>;
            copy->SetSourceTime(_sourceTime) ;
            copy->Reserve(); *(copy->_data) = T(*_data);
            return copy;
        }
        virtual void CopyTo(core::icvDataObject* dst) ICV_OVERRIDE
        {

            icvStructureData<T>* temp=static_cast< icvStructureData<T>*>(dst);
            temp->SetSourceTime(_sourceTime);
            temp->Reserve(); 
            *(temp->_data) = T(*_data);


        }

         virtual std::string Print() ICV_OVERRIDE
         {
             return "not implemented";
         }

        operator reference () { return *_data; }
        operator const_reference () const { return *_data; }

        T & getvalue () { return *_data; }

        T* operator->() { return _data; }
        const T* operator->() const { return _data; }

        icvStructureData<T>& operator = (const T& data)
        {
            *_data = data;
            return *this;
        }


        void setvalue( T& data)
        { 
         
            Reserve();          
            _data = &data;          
            this->SetSourceTime(icvTime::now_us().time_since_epoch().count());
        }
        void setvalue( T& data,const time_t timestamp)
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
        T * _data = ICV_NULLPTR;
       vector<T> _packdata;
       //T temp_data;

    };

    //#define _ICV_DECLARE_BASIC_DATA(type) typedef icvStructureData<type> icv##type##Data;
    //ICV_BASIC_TYPES_TEMPLATE(_ICV_DECLARE_BASIC_DATA);
   // #undef _ICV_DECLARE_BASIC_DATA

    // Common uses
    // typedef icvStructureData<IndexType> icvIndexData;
    // typedef icvStructureData<int>       icvIntData;
    // typedef icvStructureData<double>    icvDoubleData;
}}

#endif // icvStructureData_hxx
