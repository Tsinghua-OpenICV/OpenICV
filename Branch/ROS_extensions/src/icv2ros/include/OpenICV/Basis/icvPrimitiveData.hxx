#ifndef icvPrimitiveData_hxx
#define icvPrimitiveData_hxx

#include <boost/static_assert.hpp>
#include <boost/type_traits/is_pod.hpp>
#include <msgpack.hpp>
#include <string>
#include "OpenICV/Core/icvDataObject.h"
#include "OpenICV/Core/icvMacros.h"

using namespace msgpack;
using namespace std;

namespace icv { namespace data
{
    // TODO: add check for both to_string and msgpack serializtion implementation for custom data
    // TODO: add this to other PrimitiveData types (array, table, tensor)
    // TODO: use member function instead of to_string function. Besides, if T doesn't support print, then just disable printing precise information
    namespace _check
    {
        template<typename T>
        struct _PrimitiveDataChecker // SFINAE
        {
            /********* FIXME: Following code seems to fail to check whether to_string is available *********
            template<typename test>
            struct _has_to_string
            {
                ::std::string _check_to_string(test input)
                {
                    return ::std::to_string(input);
                }
            };
            
            template<typename test>
            struct _has_string_conversion
            {
                ::std::string _check_to_string(test input)
                {
                    return ::std::string(input);
                }
            };

            template<typename test>
            static boost::type_traits::yes_type tester(_has_to_string<test>) {}
            template<typename test>
            static boost::type_traits::yes_type tester(_has_string_conversion<test>) {}

            template<typename test>
            static boost::type_traits::no_type tester(...) {}

            enum { value = sizeof(tester<T>(0)) == sizeof(boost::type_traits::yes_type) };
            */
            enum { value = true };
        };
    }

    template<typename T>
    class icvPrimitiveData : public icv::core::icvDataObject
    {
    private:
        BOOST_STATIC_ASSERT_MSG(boost::is_pod<T>::value, "icvPrimitiveData only support POD types");
        BOOST_STATIC_ASSERT_MSG(_check::_PrimitiveDataChecker<T>::value, "Custom type should overload std::to_string");

    public:
        typedef T              value_type;
        typedef T&             reference;
        typedef const T&       const_reference;

        icvPrimitiveData()
         {
                 Reserve()    ;

        }

        icvPrimitiveData(T& data_ini) 
        {
        Reserve();
        _data =&data_ini;
        this->SetSourceTime(icvTime::time_us());

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

      

			virtual void Serialize(std::stringstream& out, const uint32_t& version) const{ 
            
                //throw "Not implemented yet!"; 
                std::stringstream buff ;
                pack(buff, *_data);
               out <<this->GetSourceTime()<<" "<<buff.str().size()<<" " << buff.rdbuf();;
       

         }
		virtual void Deserialize(std::stringstream& in, const uint32_t& version) { 
    
       // throw "Not implemented yet!"; 
        
            size_t len;
            string buff;
            time_t timestamp;
          

            ostringstream outstring;

            in >> timestamp>>len;
            in >> outstring.rdbuf(); 
            buff= outstring.str();
       
         //ICV_LOG_INFO<<"length decoded:  "<<in.str().size()<<"  BUFF SIZE "<<buff.length()<<buff.size()<<"  BUFF deco size"<<len;

            this->SetSourceTime(timestamp);     

           if(len==0) _data=ICV_NULLPTR;
           else 
           {
            msgpack::object_handle oh =msgpack::unpack(buff.data(), len);      
            object obj = oh.get();
          
            tempdata=obj.as<T>();
            _data= &tempdata;
    //ICV_LOG_INFO<<"MSGPACK DECODE COMPLETET:  ";

           }
       
            
        }
       
            
        



        virtual core::icvDataObject* DeepCopy() ICV_OVERRIDE
        {
            icvPrimitiveData<T>* copy = new icvPrimitiveData<T>();
            copy->SetSourceTime (this->GetSourceTime() );
            copy->Reserve(); *(copy->_data) = T(*_data);
            return copy;
        }



                virtual void CopyTo(core::icvDataObject* dst) ICV_OVERRIDE
        {

           icvPrimitiveData<T>* temp=static_cast< icvPrimitiveData<T>*>(dst);
                  //         ICV_LOG_INFO << "primi";

            temp->SetSourceTime (this->GetSourceTime() );
            //ICV_LOG_INFO << "primi2";

            //temp->Reserve(); 
           // ICV_LOG_INFO << "primi3";

            *(temp->_data) = T(*_data);


        }

        virtual std::string Print() ICV_OVERRIDE
        {
            return std::to_string(*_data);
        }
        T getvalue(){return *_data;}
        operator reference () { return *_data; }
        operator const_reference () const { return *_data; }

        T* operator->() { return _data; }
        const T* operator->() const { return _data; }

         void setvalue(const T& data)
        {
           // ICV_LOG_INFO << "test data:1 " ;
           Reserve();
           *_data = data;
          //  ICV_LOG_INFO << "test data:2 " ;

            this->SetSourceTime(icvTime::time_us());


        }
        void setvalue(const T& data,const time_t timestamp)
        {
            Reserve();
           *_data = data;
            this->SetSourceTime(timestamp);


        }



        icvPrimitiveData<T> * get_Ptr()
        {

            return static_cast< icvPrimitiveData<T> *> (this);

        }
        icvPrimitiveData<T>& operator = (const T& data)
        {
            *_data = data;
          //  this->SetSourceTime(icvTime::now_us().time_since_epoch().count());

            return *this;
        }
        
        inline bool operator ==(const icvPrimitiveData<T>& rhs) const
        {
            return *_data == *(rhs._data);
        }
        inline bool operator !=(const icvPrimitiveData<T>& rhs) const
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
        MSGPACK_DEFINE(*_data);
    protected:
        T * _data = ICV_NULLPTR;
        T tempdata;
      
    };

    // #define _ICV_DECLARE_BASIC_DATA(type) typedef icvPrimitiveData<type> icv##type##Data;
    // ICV_BASIC_TYPES_TEMPLATE(_ICV_DECLARE_BASIC_DATA);
    //  #undef _ICV_DECLARE_BASIC_DATA

    // Common uses

    typedef icvPrimitiveData<std::size_t> icvIndexData;
    typedef icvPrimitiveData<std::int32_t>       icvIntData;
    typedef icvPrimitiveData<std::double_t>    icvDoubleData;
    typedef icvPrimitiveData<std::int64_t>    icvInt64Data;
   

}}

#endif // icvPrimitiveData_hxx
