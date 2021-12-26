#ifndef icvPrimitiveTableData_hxx
#define icvPrimitiveTableData_hxx

#include "OpenICV/Core/icvDataObject.h"

#include <string>
#include <vector>
#include <unordered_map>

#include <boost/static_assert.hpp>
#include <boost/type_traits/is_pod.hpp>
#include <boost/type_index.hpp>

namespace icv { namespace data
{
    using namespace icv::core;

    struct icvPrimitiveTableFieldInfo
    {
        // TODO: Make ctor private and friend to makeTableField

        IndexType ElementSize;
        // std::type_info is not copyable so it cannot be used here
        // almost same as std::type_index, boost offers better compatibility
        boost::typeindex::type_index ElementType;
    };
    typedef std::pair<const std::string, icvPrimitiveTableFieldInfo> icvPrimitiveTableField;

    // If you want to use multiple values for a field, you can use boost::array<T, N> as the ElemT
    // This is also how to support string in the table. This also means that string in the table has to be fixed-length
    template <typename ElemT>
    icvPrimitiveTableField makeTableField(std::string name)
    {
        BOOST_STATIC_ASSERT_MSG(boost::is_pod<ElemT>::value, "icvPrimitiveData only support POD types");
        icvPrimitiveTableFieldInfo field = { sizeof(ElemT), boost::typeindex::type_id<ElemT>() };
        return std::make_pair(name, field);
    }

    // This implement a table as a named list of arrays, if you want to improve data efficiency,
    // you can use icvPrimitiveArray with customized data object
    class icvPrimitiveTableData : public core::icvDataObject // TODO: add iterator support (iterator type is XIndexer)
    {
    public:
        template <typename ElemT>
        class ColumnIndexer // TODO: add iterator support
        {
        private:
            ElemT * _data = ICV_NULLPTR;

        public:
            ColumnIndexer(const icvPrimitiveTableData& ref, const std::string& column)
            {
                // typeid(const ElemT) = typeid(ElemT)
                if (boost::typeindex::type_id<ElemT>() != ref._indexer.at(column).ElementType)
                    ICV_THROW_MESSAGE("Mismatched element type!");

                _data = static_cast<ElemT*>(ref._data.at(column));
            };
            ElemT & operator[](IndexType row) { return _data[row]; }
        };

    public:
        icvPrimitiveTableData(IndexType size);
        icvPrimitiveTableData(std::initializer_list<icvPrimitiveTableField> fields);
        icvPrimitiveTableData(IndexType size, std::vector<icvPrimitiveTableField> fields);

        virtual void Reserve() ICV_OVERRIDE;
        virtual void Dispose() ICV_OVERRIDE;
        void Resize(IndexType size);

        virtual Uint64 GetActualMemorySize() ICV_OVERRIDE;

        // TODO: Not implemented yet
        virtual void Serialize(std::stringstream& out, const uint32_t& version) const { throw "Not implemented yet!"; }
        virtual void Deserialize(std::stringstream& in, const uint32_t& version) { throw "Not implemented yet!"; }

        virtual icvDataObject* DeepCopy() ICV_OVERRIDE;
        virtual void CopyTo(icvDataObject* dst)  ICV_OVERRIDE {};

        virtual std::string Print();

        IndexType GetSize() { return _size; }

        template <typename ElemT>
        ElemT & At(const std::string& column, IndexType row)
        {
            if (boost::typeindex::type_id<ElemT>() != _indexer.at(column).ElementType)
                ICV_THROW_MESSAGE("Mismatched element type!");

            ElemT* arr = static_cast<ElemT*>(_data.at(column));
            return arr[row];
        }

        template <typename ElemT>
        const ElemT & At(const std::string& column, IndexType row) const
        {
            if (boost::typeindex::type_id<ElemT>() != _indexer.at(column).ElementType)
                ICV_THROW_MESSAGE("Mismatched element type!");

            ElemT* arr = static_cast<ElemT*>(_data.at(column));
            return arr[row];
        }

        template <typename ElemT>
        ColumnIndexer<ElemT> At(const std::string& column)
        {
            return ColumnIndexer<ElemT>(*this, column);
        }

        template <typename ElemT>
        ColumnIndexer<const ElemT> At(const std::string& column) const
        {
            return ColumnIndexer<const ElemT>(*this, column);
        }

        // TODO: Implement usable operator[] on this

    protected:
        IndexType _size = 0;
        std::unordered_map<std::string, icvPrimitiveTableFieldInfo> _indexer;
        std::unordered_map<std::string, void*> _data;
    };
}}

#endif // icvPrimitiveTableData_hxx
