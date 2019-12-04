#include <boost/test/unit_test.hpp>
#include <boost/mpl/vector.hpp>

#include <iostream>

#include "OpenICV/Basis/icvPrimitiveTensorData.hxx"

BOOST_AUTO_TEST_SUITE(icvPrimitiveTensorData)

using namespace icv;
using namespace icv::data;
using namespace std;
typedef vector<IndexType> Indices;

template<typename T, T data1, T data2, T data3> struct F
{
    icv::data::icvPrimitiveTensorData<T> rowmat;
    icv::data::icvPrimitiveTensorData<T, FromLast> colmat;

    T data[3] = { data1, data2, data3 };
    IndexType size = sizeof(T);

    typedef icv::data::icvPrimitiveTensorData<T> rtype;
    typedef icv::data::icvPrimitiveTensorData<T, FromLast> ctype;

    F() : rowmat(Indices{ 2, 2 }), colmat(Indices{ 2, 2 })
    {
        BOOST_TEST_MESSAGE("Constructing tensors"); 
    }
    ~F() { rowmat.Dispose(); colmat.Dispose();}

    void fill()
    {
        rowmat.Reserve();
        rowmat.At(Indices{0, 0}) = data1;
        rowmat.At(Indices{0, 1}) = data2;
        rowmat.At(Indices{1, 0}) = data3;

        colmat.Reserve();
        colmat.At(Indices{0, 0}) = data1;
        colmat.At(Indices{0, 1}) = data2;
        colmat.At(Indices{1, 0}) = data3;
    }
};

template<typename T> struct FC
{
    icv::data::icvPrimitiveTensorData<T> rowmat;
    icv::data::icvPrimitiveTensorData<T, FromLast> colmat;

    T data[3];
    IndexType size = sizeof(T);

    typedef icv::data::icvPrimitiveTensorData<T> rtype;
    typedef icv::data::icvPrimitiveTensorData<T, FromLast> ctype;

    FC(T data1, T data2, T data3) : data{ data1, data2, data3 },
        rowmat(Indices{ 2, 2 }), colmat(Indices{ 2, 2 })
    {
        BOOST_TEST_MESSAGE("Constructing arrays");
    }
    ~FC() { rowmat.Dispose(); colmat.Dispose(); }

    void fill()
    {
        rowmat.Reserve();
        rowmat.At(Indices{0, 0}) = data[0];
        rowmat.At(Indices{0, 1}) = data[1];
        rowmat.At(Indices{1, 0}) = data[2];

        colmat.Reserve();
        colmat.At(Indices{0, 0}) = data[0];
        colmat.At(Indices{0, 1}) = data[1];
        colmat.At(Indices{1, 0}) = data[2];
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
    BOOST_CHECK_EQUAL(T::rowmat.GetSize(), 4);
    BOOST_CHECK_EQUAL(T::colmat.GetSize(), 4);
}

BOOST_FIXTURE_TEST_CASE_TEMPLATE(memoryOperation, T, Fixtures, T)
{
    BOOST_CHECK_EQUAL(T::rowmat.GetActualMemorySize(), 0);
    BOOST_CHECK_EQUAL(T::rowmat.GetActualMemorySize(), 0);
    T::rowmat.Reserve();
    T::colmat.Reserve();
    BOOST_CHECK_EQUAL(T::rowmat.GetActualMemorySize(), 4 * T::size);
    BOOST_CHECK_EQUAL(T::colmat.GetActualMemorySize(), 4 * T::size);
    T::rowmat.Dispose();
    T::colmat.Dispose();
    BOOST_CHECK_EQUAL(T::rowmat.GetActualMemorySize(), 0);
    BOOST_CHECK_EQUAL(T::colmat.GetActualMemorySize(), 0);
}

BOOST_FIXTURE_TEST_CASE_TEMPLATE(At, T, Fixtures, T)
{
    T::rowmat.Reserve();
    T::rowmat.At(Indices{0, 0}) = T::data[0];
    T::rowmat.At(Indices{0, 1}) = T::data[1];
    T::rowmat.At(Indices{1, 0}) = T::data[2];
    BOOST_CHECK_EQUAL(T::rowmat.At(Indices{0, 0}), T::data[0]);
    BOOST_CHECK_EQUAL(T::rowmat.At(Indices{0, 1}), T::data[1]);
    BOOST_CHECK_EQUAL(T::rowmat.At(Indices{1, 0}), T::data[2]);
    BOOST_CHECK_EQUAL(T::rowmat.At(0u, 0u), T::data[0]);
    BOOST_CHECK_EQUAL(T::rowmat.At(0u, 1u), T::data[1]);
    BOOST_CHECK_EQUAL(T::rowmat.At(1u, 0u), T::data[2]);

    const typename T::rtype& crmat = T::rowmat;
    BOOST_CHECK_EQUAL(crmat.At(0u, 0u), T::data[0]);
    BOOST_CHECK_EQUAL(crmat.At(0u, 1u), T::data[1]);
    BOOST_CHECK_EQUAL(crmat.At(1u, 0u), T::data[2]);

    T::colmat.Reserve();
    T::colmat.At(Indices{0, 0}) = T::data[0];
    T::colmat.At(Indices{0, 1}) = T::data[1];
    T::colmat.At(Indices{1, 0}) = T::data[2];
    BOOST_CHECK_EQUAL(T::colmat.At(Indices{0, 0}), T::data[0]);
    BOOST_CHECK_EQUAL(T::colmat.At(Indices{0, 1}), T::data[1]);
    BOOST_CHECK_EQUAL(T::colmat.At(Indices{1, 0}), T::data[2]);
    BOOST_CHECK_EQUAL(T::colmat.At(0u, 0u), T::data[0]);
    BOOST_CHECK_EQUAL(T::colmat.At(0u, 1u), T::data[1]);
    BOOST_CHECK_EQUAL(T::colmat.At(1u, 0u), T::data[2]);

    const typename T::ctype& ccmat = T::colmat;
    BOOST_CHECK_EQUAL(ccmat.At(0u, 0u), T::data[0]);
    BOOST_CHECK_EQUAL(ccmat.At(0u, 1u), T::data[1]);
    BOOST_CHECK_EQUAL(ccmat.At(1u, 0u), T::data[2]);
}

BOOST_FIXTURE_TEST_CASE_TEMPLATE(copy, T, Fixtures, T)
{
    T::fill();
    uint32_t expect_memsize = T::rowmat.GetActualMemorySize();

    // RowMajor
    typename T::rtype* rcopy = static_cast<typename T::rtype*>(T::rowmat.DeepCopy());
    BOOST_CHECK_EQUAL(rcopy->GetSize(), T::rowmat.GetSize());
    BOOST_CHECK_EQUAL(rcopy->GetActualMemorySize(), expect_memsize);

    BOOST_CHECK_EQUAL(T::rowmat.At(Indices{ 0, 0 }), rcopy->At(Indices{ 0, 0 }));
    BOOST_CHECK_EQUAL(T::rowmat.At(Indices{ 0, 1 }), rcopy->At(Indices{ 0, 1 }));
    BOOST_CHECK_EQUAL(T::rowmat.At(Indices{ 1, 0 }), rcopy->At(Indices{ 1, 0 }));

    rcopy->Dispose();
    BOOST_CHECK_EQUAL(rcopy->GetActualMemorySize(), 0);
    BOOST_CHECK_EQUAL(T::rowmat.GetActualMemorySize(), expect_memsize);

    delete rcopy;

    // ColumnMajor
    typename T::ctype* ccopy = static_cast<typename T::ctype*>(T::colmat.DeepCopy());
    BOOST_CHECK_EQUAL(ccopy->GetSize(), T::colmat.GetSize());
    BOOST_CHECK_EQUAL(ccopy->GetActualMemorySize(), expect_memsize);

    BOOST_CHECK_EQUAL(T::colmat.At(Indices{ 0, 0 }), ccopy->At(Indices{ 0, 0 }));
    BOOST_CHECK_EQUAL(T::colmat.At(Indices{ 0, 1 }), ccopy->At(Indices{ 0, 1 }));
    BOOST_CHECK_EQUAL(T::colmat.At(Indices{ 1, 0 }), ccopy->At(Indices{ 1, 0 }));

    ccopy->Dispose();
    BOOST_CHECK_EQUAL(ccopy->GetActualMemorySize(), 0);
    BOOST_CHECK_EQUAL(T::colmat.GetActualMemorySize(), expect_memsize);

    delete ccopy;
}

BOOST_AUTO_TEST_SUITE_END()
