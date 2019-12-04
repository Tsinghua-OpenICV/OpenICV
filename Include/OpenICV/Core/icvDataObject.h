#ifndef icvDataObject_h
#define icvDataObject_h

#include <cstdint>
#include <string>

#include "OpenICV/Core/icvObject.h"
#include "OpenICV/Core/icvMacros.h"
#include "boost/noncopyable.hpp"

namespace icv { namespace core
{
    class icvDataObject : public icvObject, boost::noncopyable
    {
    public:
        
        virtual void Reserve() = 0; // If the data is already initialized, then this method just return
        virtual void Dispose() = 0; // If the data is already disposed, then this method just return
        bool Is_Reserved(){return is_reserved;};
        // virtual ~icvDataObject()
        // {
        //     Dispose() cannot be called here due to constructor limitation
        //     (See https://isocpp.org/wiki/faq/strange-inheritance#calling-virtuals-from-ctors)
        //
        //     shared_ptr will take the charge of freeing the memory
        // }

        virtual Uint64 GetActualMemorySize() = 0;

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
}}

#endif // icvDataObject_h
