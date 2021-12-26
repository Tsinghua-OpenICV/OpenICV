#ifndef icvPrimitiveTensorData_hxx
#define icvPrimitiveTensorData_hxx

#include "OpenICV/Core/icvDataObject.h"

#include <vector>
#include <cstdarg>
#include <boost/static_assert.hpp>
#include <boost/type_traits/is_pod.hpp>

namespace icv { namespace data
{
    enum icvTensorIndexOrder
    {
        FromFirst, // Index from leftmost, i.e. the last index is contiguous, i.e. row major, i.e. lexicographical
        FromLast,  // Index from rightmost, i.e. the first index is contiguous, i.e column major, i.e. colexicographical
    };

    template<typename ElemT, icvTensorIndexOrder IndexOrder = FromFirst>
    class icvPrimitiveTensorData : public icv::core::icvDataObject
    {
    private:
        BOOST_STATIC_ASSERT_MSG(boost::is_pod<ElemT>::value, "icvPrimitiveTensorData only support POD types");

    public:
        icvPrimitiveTensorData(const std::vector<IndexType>& shape) :_shape(shape) {}

        IndexType GetSize() const
        { 
            IndexType total = 1;
            for (auto count : _shape)
                total *= count;
            return total;
        }
        const std::vector<IndexType>& GetShape() const { return _shape; }

        virtual void Reserve() ICV_OVERRIDE { if (!_data) _data = new ElemT[GetSize()]; }
        virtual void Dispose() ICV_OVERRIDE { if (_data) delete[] _data; _data = ICV_NULLPTR; }

        void Reshape(const std::vector<IndexType>& shape) { throw "Not Implemented yet!"; }
        
        virtual Uint64 GetActualMemorySize() ICV_OVERRIDE
        {
            if (_data) return sizeof(ElemT) * GetSize();
            else return 0;
        }

        // TODO: Not implemented yet
        virtual void Serialize(std::stringstream& out, const uint32_t& version) const { throw "Not implemented yet!"; }
        virtual void Deserialize(std::stringstream& in, const uint32_t& version) { throw "Not implemented yet!"; }
        
        virtual icv::core::icvDataObject* DeepCopy() ICV_OVERRIDE
        {
            icvPrimitiveTensorData<ElemT, IndexOrder>* copy = new icvPrimitiveTensorData<ElemT, IndexOrder>(_shape);
            copy->_sourceTime = _sourceTime;
            copy->Reserve(); std::copy(_data, _data + GetSize(), copy->_data);
            return copy;
        }

        ElemT & At(const std::vector<IndexType>& index)
        {
            if (index.size() != _shape.size())
                ICV_THROW_MESSAGE("Input indices for At() should have the same length with the shape of tensor");
                // TODO: return an iterator if length of index is smaller than shape

            IndexType tidx = 0;
            if (IndexOrder == FromFirst)
                for (IndexType i = 0; i < index.size(); i++)
                    tidx = tidx * _shape[i] + index[i];
            else
            {
                for (IndexType i = index.size() - 1; i > 0; i--) // Avoid overflow of i
                    tidx = tidx * _shape[i] + index[i];
                tidx = tidx * _shape[0] + index[0];
            }

            return _data[tidx];
        }

        const ElemT & At(const std::vector<IndexType>& index) const
        {
            return const_cast<icvPrimitiveTensorData&>(*this).At(index);
        }

        // Note: use 'u' literal when calling this function
        template<typename... T>
        ElemT & At(T... indices)
        {
            std::vector<IndexType> ids{ indices... };
            return At(ids);
        }

        template<typename... T>
        const ElemT & At(T... indices) const
        {
            return const_cast<icvPrimitiveTensorData&>(*this).At(indices...);
        }

        virtual std::string Print() ICV_OVERRIDE
        {
            throw "Not implemented yet!";
        }

    protected:
        std::vector<IndexType> _shape;
        ElemT* _data = ICV_NULLPTR;
    };

    #define _ICV_DECLARE_TENSOR_DATA(type) typedef icvPrimitiveTensorData<type> icv##type##TensorData;
    ICV_BASIC_TYPES_TEMPLATE(_ICV_DECLARE_TENSOR_DATA);
    #undef _ICV_DECLARE_TENSOR_DATA

    // Common uses
    typedef icvPrimitiveTensorData<IndexType> icvIndexTensorData;
    typedef icvPrimitiveTensorData<int>       icvIntTensorData;
    typedef icvPrimitiveTensorData<double>    icvDoubleTensorData;
}}

#endif // icvPrimitiveTensorData_hxx
