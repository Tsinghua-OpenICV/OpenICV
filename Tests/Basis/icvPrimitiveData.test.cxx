#include <boost/test/unit_test.hpp>
#include <boost/mpl/vector.hpp>

#include <iostream>

struct TestStruct
{
    unsigned long long a;
    bool b;
    char c[10];
};

namespace std
{
    std::string to_string(const TestStruct& data)
    {
        return to_string(data.a) + ", " + to_string(data.b) + ", " + data.c;
    }
}

#include "OpenICV/Basis/icvPrimitiveData.hxx"

BOOST_AUTO_TEST_SUITE(icvPrimitiveData)

using namespace icv;
using namespace icv::data;

template<typename T, T tdata> struct F
{
    icv::data::icvPrimitiveData<T> obj;
    T data = tdata;
    IndexType size = sizeof(T);
    typedef icv::data::icvPrimitiveData<T> type;

    ~F() { obj.Dispose(); }

    void fill()
    {
        obj.Reserve();
        obj = data;
    }
};

template<typename T> struct FC
{
    icv::data::icvPrimitiveData<T> obj;
    T data;
    IndexType size = sizeof(T);
    typedef icv::data::icvPrimitiveData<T> type;

    FC(T tdata) : data(tdata) {}
    ~FC() { obj.Dispose(); }

    void fill()
    {
        obj.Reserve();
        obj = data;
    }
};

struct FBoolean : F<Boolean, true> {};
struct FInt8 : F<Int8, -128> {};
struct FInt16 : F<Int16, -32768> {};
struct FInt32 : F<Int32, -2147483648> {};
struct FInt64 : F<Int64, -9223372036854775808L> {};
struct FUint8 : F<Uint8, 255> {};
struct FUint16 : F<Uint16, 65535> {};
struct FUint32 : F<Uint32, 4294967295> {};
struct FUint64 : F<Uint64, 18446744073709551615ULL> {};
struct FFloat32 : FC<Float32> { FFloat32() : FC<Float32>(-1.6f) {} };
struct FFloat64 : FC<Float64> { FFloat64() : FC<Float64>(-1.6) {} };
struct FDuration32 : FC<Duration32> { FDuration32() : FC<Duration32>(4294967295s) {} };
struct FDuration64 : FC<Duration64> { FDuration64() : FC<Duration64>(18446744073709551615ns) {} };
struct FIndexType : F<IndexType, 7> {};

#define _ICV_DECLARE_DATA_TEST_TYPE(type) F##type,
typedef boost::mpl::vector<
    ICV_BASIC_TYPES_TEMPLATE(_ICV_DECLARE_DATA_TEST_TYPE) FIndexType
> Fixtures;

BOOST_FIXTURE_TEST_CASE_TEMPLATE(memoryOperation, T, Fixtures, T)
{
    BOOST_CHECK_EQUAL(T::obj.GetActualMemorySize(), 0);
    T::obj.Reserve();
    BOOST_CHECK_EQUAL(T::obj.GetActualMemorySize(), T::size);
    T::obj.Dispose();
    BOOST_CHECK_EQUAL(T::obj.GetActualMemorySize(), 0);
}

BOOST_FIXTURE_TEST_CASE_TEMPLATE(assign, T, Fixtures, T)
{
    T::obj.Reserve();
    T::obj = T::data;
    BOOST_CHECK_EQUAL(T::obj, T::data);

    const typename T::type& cobj = T::obj;
    BOOST_CHECK_EQUAL(cobj, T::data);
}

BOOST_FIXTURE_TEST_CASE_TEMPLATE(copy, T, Fixtures, T)
{
    T::fill();
    uint32_t expect_memsize = T::obj.GetActualMemorySize();

    typename T::type* copy = static_cast<typename T::type*>(T::obj.DeepCopy());
    BOOST_CHECK_EQUAL(*copy, T::obj);
    BOOST_CHECK_EQUAL(copy->GetActualMemorySize(), expect_memsize);

    copy->Dispose();
    BOOST_CHECK_EQUAL(copy->GetActualMemorySize(), 0);
    BOOST_CHECK_EQUAL(T::obj.GetActualMemorySize(), expect_memsize);

    delete copy;
}

BOOST_AUTO_TEST_CASE(custom_data_type)
{
    icv::data::icvPrimitiveData<TestStruct> obj;
    obj.Reserve();
    obj = TestStruct{1ULL, true, "Test"};

    BOOST_CHECK_EQUAL(((TestStruct)obj).a, 1ULL);
    BOOST_CHECK_EQUAL(((TestStruct)obj).b, true);
    BOOST_CHECK_EQUAL(((TestStruct)obj).c, "Test");

    obj.Dispose();
}

BOOST_AUTO_TEST_SUITE_END()
