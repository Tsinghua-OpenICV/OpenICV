#include <boost/test/unit_test.hpp>
#include <boost/mpl/vector.hpp>

#include <iostream>

#include "OpenICV/Basis/icvPrimitiveArrayData.hxx"

BOOST_AUTO_TEST_SUITE(icvPrimitiveArrayData)

using namespace icv;
using namespace icv::data;

template<typename T, T data1, T data2, T data3> struct F
{
    icv::data::icvPrimitiveArrayData<T> arr;
    T data[3] = { data1, data2, data3 };
    IndexType size = sizeof(T);
    typedef icv::data::icvPrimitiveArrayData<T> type;

    F() : arr(3) { BOOST_TEST_MESSAGE("Constructing arrays"); }
    ~F() { arr.Dispose(); }

    void fill()
    {
        arr.Reserve();
        arr[0] = data1;
        arr[1] = data2;
        arr[2] = data3;
    }
};

template<typename T> struct FC
{
    icv::data::icvPrimitiveArrayData<T> arr;
    T data[3];
    IndexType size = sizeof(T);
    typedef icv::data::icvPrimitiveArrayData<T> type;

    FC(T data1, T data2, T data3) : arr(3), data { data1, data2, data3 }
    {
        BOOST_TEST_MESSAGE("Constructing arrays"); 
    }
    ~FC() { arr.Dispose(); }

    void fill()
    {
        arr.Reserve();
        arr[0] = data[0];
        arr[1] = data[1];
        arr[2] = data[2];
    }
};

struct FBoolean : F<Boolean, true, false, true> {};
struct FInt8 : F<Int8, -128, 0, 127> {};
struct FInt16 : F<Int16, -32768, 0, 32767> {};
struct FInt32 : F<Int32, -2147483648, 0, 2147483647> {};
struct FInt64 : F<Int64, -9223372036854775808L, 0, 9223372036854775807L> {};
struct FUint8 : F<Uint8, 0, 1, 255> {};
struct FUint16 : F<Uint16, 0, 1, 65535> {};
struct FUint32 : F<Uint32, 0, 1, 4294967295> {};
struct FUint64 : F<Uint64, 0, 1, 18446744073709551615ULL> {};
struct FFloat32 : FC<Float32> { FFloat32() : FC<Float32>(-1.6f, 0.0f, 1.6f) {} };
struct FFloat64 : FC<Float64> { FFloat64() : FC<Float64>(-1.6, 0.0, 1.6) {} };
struct FDuration32 : FC<Duration32> { FDuration32() : FC<Duration32>(0s, 1s, 4294967295s) {} };
struct FDuration64 : FC<Duration64> { FDuration64() : FC<Duration64>(0ns, 1ns, 18446744073709551615ns) {} };
struct FIndexType : F<IndexType, 0, 1, 7> {};

#define _ICV_DECLARE_ARRAY_TEST_TYPE(type) F##type,
typedef boost::mpl::vector<
    ICV_BASIC_TYPES_TEMPLATE(_ICV_DECLARE_ARRAY_TEST_TYPE) FIndexType
> Fixtures;

BOOST_FIXTURE_TEST_CASE_TEMPLATE(constructor, T, Fixtures, T)
{
    BOOST_CHECK_EQUAL(T::arr.GetSize(), 3);
}

BOOST_FIXTURE_TEST_CASE_TEMPLATE(memoryOperation, T, Fixtures, T)
{
    BOOST_CHECK_EQUAL(T::arr.GetActualMemorySize(), 0);
    BOOST_CHECK_EQUAL(T::arr.GetSize(), 3);
    T::arr.Reserve();
    BOOST_CHECK_EQUAL(T::arr.GetActualMemorySize(), 3 * T::size);
    BOOST_CHECK_EQUAL(T::arr.GetSize(), 3);
    T::arr.Dispose();
    BOOST_CHECK_EQUAL(T::arr.GetActualMemorySize(), 0);
    BOOST_CHECK_EQUAL(T::arr.GetSize(), 3);
}

BOOST_FIXTURE_TEST_CASE_TEMPLATE(indexer, T, Fixtures, T)
{
    T::arr.Reserve();
    T::arr[0] = T::data[0];
    T::arr[1] = T::data[1];
    T::arr[2] = T::data[2];
    BOOST_CHECK_EQUAL(T::arr[0], T::data[0]);
    BOOST_CHECK_EQUAL(T::arr[1], T::data[1]);
    BOOST_CHECK_EQUAL(T::arr[2], T::data[2]);

    const typename T::type& carr = T::arr;
    BOOST_CHECK_EQUAL(carr[0], T::data[0]);
    BOOST_CHECK_EQUAL(carr[1], T::data[1]);
    BOOST_CHECK_EQUAL(carr[2], T::data[2]);
}

BOOST_FIXTURE_TEST_CASE_TEMPLATE(At, T, Fixtures, T)
{
    T::arr.Reserve();
    T::arr.At(0) = T::data[0];
    T::arr.At(1) = T::data[1];
    T::arr.At(2) = T::data[2];
    BOOST_CHECK_EQUAL(T::arr.At(0), T::data[0]);
    BOOST_CHECK_EQUAL(T::arr.At(1), T::data[1]);
    BOOST_CHECK_EQUAL(T::arr.At(2), T::data[2]);

    const typename T::type& carr = T::arr;
    BOOST_CHECK_EQUAL(carr.At(0), T::data[0]);
    BOOST_CHECK_EQUAL(carr.At(1), T::data[1]);
    BOOST_CHECK_EQUAL(carr.At(2), T::data[2]);
}

BOOST_FIXTURE_TEST_CASE_TEMPLATE(GetSize, T, Fixtures, T)
{
    T::fill();
    BOOST_CHECK_EQUAL(T::arr.GetSize(), 3);
}

BOOST_FIXTURE_TEST_CASE_TEMPLATE(Resize, T, Fixtures, T)
{
    T::fill();
    T::arr.Resize(2);
    BOOST_CHECK_EQUAL(T::arr.GetSize(), 2);
    BOOST_CHECK_EQUAL(T::arr[0], T::data[0]);
    BOOST_CHECK_EQUAL(T::arr[1], T::data[1]);

    T::arr.Resize(4);
    BOOST_CHECK_EQUAL(T::arr.GetSize(), 4);
    T::arr.At(2) = T::data[0];
    T::arr.At(3) = T::data[1];
    BOOST_CHECK_EQUAL(T::arr[2], T::data[0]);
    BOOST_CHECK_EQUAL(T::arr[3], T::data[1]);
}

BOOST_FIXTURE_TEST_CASE_TEMPLATE(iterator, T, Fixtures, T)
{
    T::fill();
    int counter = 0;
    for(auto data : T::arr)
        BOOST_CHECK_EQUAL(data, T::data[counter++]);
}

BOOST_FIXTURE_TEST_CASE_TEMPLATE(copy, T, Fixtures, T)
{
    T::fill();
    uint32_t expect_memsize = T::arr.GetActualMemorySize();

    typename T::type* copy = static_cast<typename T::type*>(T::arr.DeepCopy());
    BOOST_CHECK_EQUAL(copy->GetSize(), T::arr.GetSize());
    BOOST_CHECK_EQUAL(copy->GetActualMemorySize(), expect_memsize);
    BOOST_CHECK_EQUAL_COLLECTIONS(copy->begin(), copy->end(),
        T::arr.begin(), T::arr.end());

    copy->Dispose();
    BOOST_CHECK_EQUAL(copy->GetActualMemorySize(), 0);
    BOOST_CHECK_EQUAL(T::arr.GetActualMemorySize(), expect_memsize);

    delete copy;
}

BOOST_AUTO_TEST_SUITE_END()
