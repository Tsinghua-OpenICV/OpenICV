#ifndef icvPrimitiveArrayData_hxx
#define icvPrimitiveArrayData_hxx

#include "OpenICV/Core/icvDataObject.h"

#include <cstdlib>
#include <boost/static_assert.hpp>
#include <boost/type_traits/is_pod.hpp>

namespace icv { namespace data
{
    // TODO: Implement bool array as bitarray
    // TODO: Deal with object alignment issue like in VTK and Eigen (also in tensor data)
    template<typename ElemT>
    class icvPrimitiveArrayData : public icv::core::icvDataObject
    {
    private:
        BOOST_STATIC_ASSERT_MSG(boost::is_pod<ElemT>::value, "icvPrimitiveArrayData only support POD types");

    public:
        icvPrimitiveArrayData(IndexType size) : _size(size) {}

        // `malloc` and `free` are used instead of `new` to support `realloc`
        virtual void Reserve() ICV_OVERRIDE
        {
            if (!_data) 
            {
                _data = static_cast<ElemT*>(std::malloc(_size * sizeof(ElemT)));
                is_reserved=true;
            }
            if (!_data) ICV_THROW_MESSAGE("Bad allocation");
        }
        virtual void Dispose() ICV_OVERRIDE
        {
            if (_data) std::free(_data);
            _data = ICV_NULLPTR;
        }
        void Resize(IndexType size)
        { 
            if (!_data) _data = static_cast<ElemT*>(std::malloc(size * sizeof(ElemT)));
            else _data = static_cast<ElemT*>(std::realloc(_data, size * sizeof(ElemT)));
            if (!_data) ICV_THROW_MESSAGE("Bad allocation");
            else _size = size;
        }

        virtual Uint64 GetActualMemorySize() ICV_OVERRIDE
        {
            if (_data) return sizeof(ElemT) * _size;
            else return 0;
        }

        // TODO: Not implemented yet
        virtual void Serialize(std::ostream& out, const uint32_t& version)  { throw "Not implemented yet!"; }
        virtual void Deserialize(std::istream& in, const uint32_t& version) { throw "Not implemented yet!"; }

        virtual icv::core::icvDataObject* DeepCopy() ICV_OVERRIDE
        {
            icvPrimitiveArrayData<ElemT>* copy = new icvPrimitiveArrayData<ElemT>(_size);
            copy->_sourceTime = _sourceTime;
            copy->_size = _size;
            copy->Reserve(); std::copy(begin(), end(), copy->begin());
            return copy;
        }

        virtual std::string Print() ICV_OVERRIDE
        {
            throw "Not implemented yet!";
        }

        IndexType GetSize() { return _size; }

        ElemT & At(IndexType index)
        {
            if (index > _size) ICV_THROW_MESSAGE("out of range");
            return _data[index];
        }
        const ElemT & At(IndexType index) const
        {
            if (index > _size) ICV_THROW_MESSAGE("out of range");
            return _data[index];
        }

        /*********** iterator support (from Boost.array) **********/
        typedef ElemT          value_type;
        typedef ElemT*         iterator;
        typedef const ElemT*   const_iterator;
        typedef ElemT&         reference;
        typedef const ElemT&   const_reference;
        typedef IndexType      size_type;
        typedef IndexDiffType  difference_type;

        iterator        begin() { return _data; }
        const_iterator  begin() const { return _data; }
        const_iterator cbegin() const { return _data; }

        iterator        end() { return _data + _size; }
        const_iterator  end() const { return _data + _size; }
        const_iterator cend() const { return _data + _size; }

        typedef std::reverse_iterator<iterator> reverse_iterator;
        typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

        reverse_iterator rbegin() { return reverse_iterator(end()); }
        const_reverse_iterator rbegin() const { return const_reverse_iterator(end()); }
        const_reverse_iterator crbegin() const { return const_reverse_iterator(end()); }
        
        reverse_iterator rend() { return reverse_iterator(begin()); }
        const_reverse_iterator rend() const { return const_reverse_iterator(begin()); }
        const_reverse_iterator crend() const { return const_reverse_iterator(begin()); }
        
        reference operator[](size_type i) { return At(i); }
        const_reference operator[](size_type i) const { return At(i); }

        /**********************************************************/

    protected:
        IndexType _size;
        ElemT * _data = ICV_NULLPTR;
    };

    #define _ICV_DECLARE_ARRAY_DATA(type) typedef icvPrimitiveArrayData<type> icv##type##ArrayData;
    ICV_BASIC_TYPES_TEMPLATE(_ICV_DECLARE_ARRAY_DATA);
    #undef _ICV_DECLARE_ARRAY_DATA

    // Common uses
    typedef icvPrimitiveArrayData<IndexType> icvIndexArrayData;
    typedef icvPrimitiveArrayData<int>       icvIntArrayData;
    typedef icvPrimitiveArrayData<double>    icvDoubleArrayData;
}}

#endif // icvPrimitiveArrayData_hxx
