#ifndef icvPrimitiveData_hxx
#define icvPrimitiveData_hxx

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

        icvPrimitiveData() {
             _data = new T; is_reserved=true;
        }

        virtual void Reserve() ICV_OVERRIDE 
        { 
           // if (!_data) _data = new T; 
        }
        virtual void Dispose() ICV_OVERRIDE { if (_data) delete _data; _data = ICV_NULLPTR; }

        virtual Uint64 GetActualMemorySize() ICV_OVERRIDE 
        { 
            if (_data) return sizeof(T);
            else return 0;
        }

        // TODO: Not implemented yet

      

		virtual void Serialize(std::ostream& out, const uint32_t& version) const{ 
            
                //throw "Not implemented yet!"; 
                sbuffer buff;
                pack(buff, *_data);
             //   ICV_LOG_INFO<<"time recorded:  "<<this->GetSourceTime();
             //    ICV_LOG_INFO<<"length recorded:  "<<buff.size();

                out <<this->GetSourceTime()<<" "<<buff.size() << buff.data();
         }
		virtual void Deserialize(std::istream& in, const uint32_t& version) { 
    
       // throw "Not implemented yet!"; 
            unpacked result;
            size_t len;
            string buff;
            time_t timestamp;

            in >> timestamp>>len >> buff;  
          // ICV_LOG_INFO<<"length decoded:  ";

            this->SetSourceTime(timestamp);     
      //  ICV_LOG_INFO<<"time decoded:  "<<this->GetSourceTime();

           if(len==0) _data=ICV_NULLPTR;
           else 
           {

            unpack(&result, buff.c_str(), len);
            object obj = result.get();
            T temp_data=obj.as<T>();
            _data= &temp_data;

           }
            
            
        }



        virtual core::icvDataObject* DeepCopy() ICV_OVERRIDE
        {
            icvPrimitiveData<T>* copy = new icvPrimitiveData<T>;
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

        operator reference () { return *_data; }
        operator const_reference () const { return *_data; }

        T* operator->() { return _data; }
        const T* operator->() const { return _data; }

         void setoutvalue(const T& data)
        {
           // ICV_LOG_INFO << "test data:1 " ;
           *_data = data;
          //  ICV_LOG_INFO << "test data:2 " ;

            this->SetSourceTime(SyncClock::now_us().time_since_epoch().count());


        }
                void setoutvalue(const T& data,const time_t timestamp)
        {
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
          //  this->SetSourceTime(SyncClock::now_us().time_since_epoch().count());

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

    protected:
        T * _data = ICV_NULLPTR;
      //  MSGPACK_DEFINE(_data);
    };

    #define _ICV_DECLARE_BASIC_DATA(type) typedef icvPrimitiveData<type> icv##type##Data;
    ICV_BASIC_TYPES_TEMPLATE(_ICV_DECLARE_BASIC_DATA);
    #undef _ICV_DECLARE_BASIC_DATA

    // Common uses
    typedef icvPrimitiveData<IndexType> icvIndexData;
    typedef icvPrimitiveData<int>       icvIntData;
    typedef icvPrimitiveData<double>    icvDoubleData;
}}

#endif // icvPrimitiveData_hxx
