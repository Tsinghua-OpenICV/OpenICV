#ifndef icvStringData_hxx
#define icvStringData_hxx

#include "OpenICV/Basis/icvPrimitiveArrayData.hxx"

namespace icv { namespace data
{
    template <typename CharT>
    class icvStringData : public icvPrimitiveArrayData<CharT>
    {
    public:
        icvStringData(IndexType max_len) : icvPrimitiveArrayData(max_len) {}
        icvStringData(const std::basic_string<CharT>& data) :
            icvPrimitiveArrayData(data.size())
        {
            Reserve();
            *this = data;
        }

        operator std::basic_string<CharT>()
        {
            return std::basic_string(_data);
        }

        icvStringData<CharT>& operator = (const std::basic_string<CharT>& data)
        {
            _length = data.size();
            if (_length <= _size)
            {
                std::copy(data.begin(), data.end(), _data);
                std::fill(begin() + _length, end(), 0);
            }
            else
            {
                // Truncate tail
                std::copy(data.begin(), data.begin() + _size, _data);
            }
            return *this;
        }

        icvStringData<CharT>& operator = (const CharT* data)
        {
            _length = 0;
            while (data[index] != 0)
            {
                _data[_length] = data[_length];
                _length++;
            }
        }

        // Return the lenght of string. ( Different from GetSize() )
        IndexType GetLength() const { return _length; }

    protected:
        IndexType _length;
    };

    // TODO: char16_t and char32_t is defined in C++ 11, make it compatible to uint16_t, uint32_t
    typedef icvStringData<char>  icvString8Data;
    typedef icvStringData<char16_t> icvString16Data;
    typedef icvStringData<char32_t> icvString32Data;
}}

#endif // icvString_hxx
