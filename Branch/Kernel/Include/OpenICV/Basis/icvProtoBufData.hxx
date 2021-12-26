#ifndef icvProtoBufData_hxx
#define icvProtoBufData_hxx

#include <boost/static_assert.hpp>
#include <boost/type_traits/is_pod.hpp>
// #include <msgpack.hpp>
#include <string>
#include "OpenICV/Core/icvDataObject.h"
#include "icvProtoBufData.pb/icvPBData.pb.h"

using namespace std;

namespace icv { namespace data
{
    // TODO: add check for both to_string and msgpack serializtion implementation for custom data
    // TODO: add this to other PrimitiveData types (array, table, tensor)
    // TODO: use member function instead of to_string function. Besides, if T doesn't support print, then just disable printing precise information
   
    // template<typename T>
    template < typename T, typename = typename std::enable_if<std::is_base_of<google::protobuf::Message, T>::value>::type>
    class icvProtoBufData : public icv::core::icvDataObject
    {

    public:
        typedef T              value_type;
        typedef T&             reference;
        typedef const T&       const_reference;

        icvProtoBufData() {
            Reserve();
            data_type=ICVPORTOBUFDATA;
        }

        icvProtoBufData(value_type & data_ini) {
        _data =&data_ini;
        data_type=ICVPORTOBUFDATA;
        //setvalue(&data_ini);
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


	// 		virtual void Serialize(std::stringstream& out, const uint32_t& version) const{ 
            
    //             //throw "Not implemented yet!"; 
    //             std::stringstream buff ;
    //             pack(buff, *_data);
    //            out <<this->GetSourceTime()<<" "<<buff.str().size()<<" " << buff.str().data();
       

    //      }
	// 	virtual void Deserialize(std::stringstream& in, const uint32_t& version) { 
    
    //    // throw "Not implemented yet!"; 
        
    //         size_t len;
    //         string buff;
    //         time_t timestamp;
          

    //         in >> timestamp>>len>>buff;
       
    //      //ICV_LOG_INFO<<"length decoded:  "<<in.str().size()<<"  BUFF SIZE "<<buff.length()<<buff.size()<<"  BUFF deco size"<<len;

    //         this->SetSourceTime(timestamp);     

    //        if(len==0) _data=ICV_NULLPTR;
    //        else 
    //        {
    //         msgpack::object_handle oh =msgpack::unpack(buff.data(), len);      
    //         object obj = oh.get();
          
    //         tempdata=obj.as<T>();
    //         _data= &tempdata;
    // //ICV_LOG_INFO<<"MSGPACK DECODE COMPLETET:  ";

    //        }
       
            
    //     }
    
        // virtual bool checkType()
        // {
        //     return std::is_base_of<google::protobuf::Message, T>::value;
        // }

        virtual void Serialize(std::stringstream& out, const uint32_t& version)const 
        {
            GOOGLE_PROTOBUF_VERIFY_VERSION;
            icvPBDataPkg::icvPBDataMsg serPbMsg;
            serPbMsg.mutable_buffer()->PackFrom(*_data);
            serPbMsg.set_timestamp(this->GetSourceTime());
            serPbMsg.set_len(sizeof(*_data));
            serPbMsg.SerializeToOstream(&out);
        }

    //    virtual void Serialize(std::stringstream& out, const uint32_t& version)const { 
    //         std::stringstream buff ;
    //         pack(buff, *_data);
    //         out <<this->GetSourceTime()<<" "<<buff.str().size()<<" " << buff.rdbuf();
    //      }


        // typename std::enable_if<std::is_base_of<google::protobuf::Message, T>::value,void>::type
		virtual void Deserialize(std::stringstream& in, const uint32_t& version) 
        { 
            GOOGLE_PROTOBUF_VERIFY_VERSION;
            icvPBDataPkg::icvPBDataMsg serPbMsg;
            serPbMsg.ParseFromIstream(&in);

            this->SetSourceTime(serPbMsg.timestamp());
            if (serPbMsg.len() == 0) 
            {
                Dispose();
            }
            else 
            {
                Reserve();
                serPbMsg.mutable_buffer()->UnpackTo(_data);
            }
        }

        // virtual void Deserialize(std::stringstream& in,
        //                          const uint32_t& version) 
        //{
        //     // throw "Not implemented yet!";
        //     unpacked result;
        //     size_t len;
        //     string buff;
        //     time_t timestamp;
        //     ostringstream outstring;

        //     in >> timestamp >> len;
        //     in >> outstring.rdbuf();
        //     buff = outstring.str();
        //     this->SetSourceTime(timestamp);
        //     if (len == 0)
        //         _data = ICV_NULLPTR;
        //     else {
        //         msgpack::object_handle oh = msgpack::unpack(buff.data(), len);
        //         object obj = oh.get();

        //         obj.convert(temp_data);
        //         _data = &temp_data;
        //     }
        // }

       icvProtoBufData<T>* get_Ptr(){static_cast< icvProtoBufData<T>*> (this); }

        virtual core::icvDataObject* DeepCopy() ICV_OVERRIDE
        {
            icvProtoBufData<T>* copy = new icvProtoBufData<T>;
            copy->SetSourceTime(_sourceTime) ;
            copy->Reserve(); *(copy->_data) = T(*_data);
            return copy;
        }
        virtual void CopyTo(core::icvDataObject* dst) ICV_OVERRIDE
        {

            icvProtoBufData<T>* temp=static_cast< icvProtoBufData<T>*>(dst);
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
        T* getvaluePtr () { return _data; }
        T* operator->() { return _data; }
        const T* operator->() const { return _data; }

        icvProtoBufData<T>& operator = (const T& data)
        {
            *_data = data;
            return *this;
        }


        void setvalue( T& data)
        { 
         
            Reserve();          
            _data = &data;          
            this->SetSourceTime(icvTime::time_us());
        }
        void setvalue( T& data,const time_t timestamp)
        {
            Reserve();
            _data = &data;
            this->SetSourceTime(timestamp);
        }
        
        inline bool operator ==(const icvProtoBufData<T>& rhs) const
        {
            return *_data == *(rhs._data);
        }
        inline bool operator !=(const icvProtoBufData<T>& rhs) const
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
        // MSGPACK_DEFINE(* _data);

    protected:
        T * _data = ICV_NULLPTR;
       vector<T> _packdata;
        T temp_data; 

    };

    //#define _ICV_DECLARE_BASIC_DATA(type) typedef icvProtoBufData<type> icv##type##Data;
    //ICV_BASIC_TYPES_TEMPLATE(_ICV_DECLARE_BASIC_DATA);
   // #undef _ICV_DECLARE_BASIC_DATA

    // Common uses
    // typedef icvProtoBufData<IndexType> icvIndexData;
    // typedef icvProtoBufData<int>       icvIntData;
    // typedef icvProtoBufData<double>    icvDoubleData;
}}

#endif // icvProtoBufData_hxx

