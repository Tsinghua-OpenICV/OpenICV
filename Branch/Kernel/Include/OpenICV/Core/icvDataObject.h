#ifndef icvDataObject_h
#define icvDataObject_h

#include <cstdint>
#include <string>

#include "OpenICV/Core/icvObject.h"
#include "OpenICV/Core/icvMacros.h"
#include "boost/noncopyable.hpp"
#define ICV_CONFIG_POINTERS
#include "OpenICV/Core/icvConfig.h" 
#undef ICV_CONFIG_POINTERS
namespace icv { namespace core
{
    class icvDataObject : public icvObject, boost::noncopyable
    {
    public:
        typedef icv_shared_ptr<icvDataObject> Ptr;

        virtual void Reserve() = 0; // If the data is already initialized, then this method just return
        virtual void Dispose() = 0; // If the data is already disposed, then this method just return
        bool Is_Reserved(){return is_reserved;};
        bool is_not_empty(){return !empty;};
          ICV_ENUM_CLASS icvDataSerializeType
        {
        MSGPACK, // MSPACK PACK
        PythonStruct, // PYTHON STRUCT PACK

        };
        enum DATATYPE
        {
              ICVSTRUCTUREDATA=1, ICVCVMATDATA, ICVPOINTCLOUDDATA, ICVPRIMITIVEDATA, ICVPORTOBUFDATA
        };
       
        // virtual ~icvDataObject()
        // {
        //     Dispose() cannot be called here due to constructor limitation
        //     (See https://isocpp.org/wiki/faq/strange-inheritance#calling-virtuals-from-ctors)
        //
        //     shared_ptr will take the charge of freeing the memory
        // }
        bool empty=true;
        virtual Uint64 GetActualMemorySize() = 0;

        ICV_PROPERTY_GETSET(SourceTime, _sourceTime, time_t)

        // TODO: serialize timestamp separately
        virtual void Serialize(std::stringstream& out, const uint32_t& version) const = 0;
        virtual void Deserialize(std::stringstream& in, const uint32_t& version) = 0;

        // TODO: Split initialization and data copy to help derived class implement this
        // TODO: Return shared pointer instead?
        virtual icvDataObject* DeepCopy() = 0;
        // ShallowCopy is not allowed for data object, please copy the pointer to data object directly
        // icvDataObject* ShallowCopy() = delete;
        
        virtual void CopyTo(icvDataObject* dst) = 0;
      // template <typename T>  void setoutputdata(icvDataObject* dst){};

        virtual std::string Print() = 0; // For information display

        // Downcast this data object to T
        template <typename T> inline T& As() { return *(static_cast<T*>(this)); }
        template <typename T> inline const T& As() const { return *(static_cast<T*>(this)); }
        int Get_Data_Type(){return data_type;}
    protected:
        time_t _sourceTime=0;
        bool is_reserved=false;
        icvDataSerializeType serial_tag=icvDataSerializeType::MSGPACK;
         enum DATATYPE data_type;
    };
}}

#endif // icvDataObject_h
