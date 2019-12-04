#include "OpenICV/Basis/icvPrimitiveTableData.hxx"

#include <boost/type_traits/is_pod.hpp>

#include <cstdarg>
#include <cstdlib>

using namespace std;

namespace icv { namespace data
{
    icvPrimitiveTableData::icvPrimitiveTableData(IndexType size) : _size(size) {}

    icvPrimitiveTableData::icvPrimitiveTableData(initializer_list<icvPrimitiveTableField> fields)
    {
        for (auto iter = fields.begin(); iter != fields.end(); iter++)
        {
            _indexer.emplace(*iter);
            _data[iter->first] = ICV_NULLPTR;
        }
    }

    icvPrimitiveTableData::icvPrimitiveTableData(IndexType size, vector<icvPrimitiveTableField> fields)
        : _size(size)
    {
        for (auto iter = fields.begin(); iter != fields.end(); iter++)
        {
            _indexer.emplace(*iter);
            _data[iter->first] = ICV_NULLPTR;
        }
    }

    void icvPrimitiveTableData::Reserve()
    {
        if (_size == 0) return;
        for (auto iter = _indexer.begin(); iter != _indexer.end(); iter++)
        {
            if (!_data[iter->first])
                _data[iter->first] = malloc(iter->second.ElementSize * _size);
            if (!_data[iter->first]) ICV_THROW_MESSAGE("Bad allocation");
        }
    }

    void icvPrimitiveTableData::Dispose()
    {
        for (auto iter = _indexer.begin(); iter != _indexer.end(); iter++)
        {
            if (_data[iter->first])
            {
                free(_data[iter->first]);
                _data[iter->first] = ICV_NULLPTR;
            }
        }
    }

    void icvPrimitiveTableData::Resize(IndexType size)
    {
        for (auto iter = _indexer.begin(); iter != _indexer.end(); iter++)
        {
            IndexType newsize = iter->second.ElementSize * size;
            if (_data[iter->first])
                _data[iter->first] = realloc(_data[iter->first], newsize);
            else _data[iter->first] = malloc(newsize);
            if (!_data[iter->first]) ICV_THROW_MESSAGE("Bad allocation");
        }
        _size = size;
    }

    Uint64 icvPrimitiveTableData::GetActualMemorySize()
    {
        uint32_t total = 0;
        for (auto iter = _indexer.begin(); iter != _indexer.end(); iter++)
        {
            if (_data[iter->first])
                total += iter->second.ElementSize * _size;
        }
        return total;
    }

    icvDataObject* icvPrimitiveTableData::DeepCopy()
    {
        icvPrimitiveTableData* copy = new icvPrimitiveTableData(_size);
        copy->_indexer = _indexer;
        copy->_size = _size;
        copy->Reserve();
        for (auto iter = _indexer.begin(); iter != _indexer.end(); iter++)
            std::memcpy(copy->_data[iter->first], _data[iter->first],
                iter->second.ElementSize * _size);
        return copy;
    }

    std::string icvPrimitiveTableData::Print()
    {
        throw "Not implemented yet!";
    }
}}
