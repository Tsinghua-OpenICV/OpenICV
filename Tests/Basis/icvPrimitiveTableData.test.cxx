#include <boost/test/unit_test.hpp>

#include <boost/array.hpp>
#include "OpenICV/Basis/icvPrimitiveTableData.hxx"

BOOST_AUTO_TEST_SUITE(icvPrimitiveTableData)

using namespace icv;
using namespace icv::data;

template<typename T, T data1, T data2, T data3> struct F
{
    typedef T type;
    T data[3] = { data1, data2, data3 };
};

template<typename T> struct FC
{
    T data[3]; typedef T type;
    FC(T data1, T data2, T data3) : data{ data1, data2, data3 } {}
};

F<Boolean, true, false, true> FBoolean;
F<Int8, -128, 0, 127> FInt8;
F<Int16, -32768, 0, 32767> FInt16;
F<Int32, -2147483648, 0, 2147483647> FInt32;
F<Int64, -9223372036854775808L, 0, 9223372036854775807L> FInt64;
F<Uint8, 0, 1, 255> FUint8;
F<Uint16, 0, 1, 65535> FUint16;
F<Uint32, 0, 1, 4294967295> FUint32;
F<Uint64, 0, 1, 18446744073709551615ULL> FUint64;
FC<Float32> FFloat32(-1.6f, 0.0f, 1.6f);
FC<Float64> FFloat64(-1.6, 0.0, 1.6);
FC<Duration32> FDuration32(0s, 1s, 4294967295s);
FC<Duration64> FDuration64(0ns, 1ns, 18446744073709551615ns);
F<IndexType, 0, 1, 7> FIndexType{};

struct Fixture
{
    icv::data::icvPrimitiveTableData table;

    #define _ICV_DECLARE_DATA_TEST_TYPE(type) makeTableField<type>(#type),

    Fixture() : table(3, {
        ICV_BASIC_TYPES_TEMPLATE(_ICV_DECLARE_DATA_TEST_TYPE)
        makeTableField<IndexType>("IndexType")
        }) {}// Init
    ~Fixture() { table.Dispose(); }

    template<typename T>
    void SetRow(std::string column, T data)
    {
        table.At<typename T::type>(column, 0) = data.data[0];
        table.At<typename T::type>(column, 1) = data.data[1];
        table.At<typename T::type>(column, 2) = data.data[2];
    }

    void fill()
    {
        table.Reserve();
        #define _ICV_DECLARE_DATA_TEST_FILL(type) SetRow(#type, F ## type);
        ICV_BASIC_TYPES_TEMPLATE(_ICV_DECLARE_DATA_TEST_FILL)
        _ICV_DECLARE_DATA_TEST_FILL(IndexType);
    }
};

BOOST_FIXTURE_TEST_CASE(constructor, Fixture)
{
    BOOST_CHECK_EQUAL(table.GetSize(), 3);
}

BOOST_FIXTURE_TEST_CASE(memoryOperation, Fixture)
{
    #define _ICV_SIZEOF_TEST_TYPE(type) sizeof(type)+
    IndexType tsize = ICV_BASIC_TYPES_TEMPLATE(_ICV_SIZEOF_TEST_TYPE) sizeof(IndexType);

    BOOST_CHECK_EQUAL(table.GetActualMemorySize(), 0);
    BOOST_CHECK_EQUAL(table.GetSize(), 3);
    table.Reserve();
    BOOST_CHECK_EQUAL(table.GetActualMemorySize(), 3 * tsize);
    BOOST_CHECK_EQUAL(table.GetSize(), 3);
    table.Dispose();
    BOOST_CHECK_EQUAL(table.GetActualMemorySize(), 0);
    BOOST_CHECK_EQUAL(table.GetSize(), 3);
}

BOOST_FIXTURE_TEST_CASE(At, Fixture)
{
    table.Reserve();
    table.At<Int8>("Int8", 0) = 16;
    table.At<Int8>("Int8", 1) = -16;
    BOOST_CHECK_EQUAL(table.At<Int8>("Int8")[0], 16);
    BOOST_CHECK_EQUAL(table.At<Int8>("Int8")[1], -16);
}

BOOST_FIXTURE_TEST_CASE(indexer, Fixture)
{
    fill();

    #define _ICV_CHECK_TEST_TABLE_DATA(type)                            \
        BOOST_CHECK_EQUAL(table.At<type>(#type)[0], F##type.data[0]);  \
        BOOST_CHECK_EQUAL(table.At<type>(#type)[1], F##type.data[1]);  \
        BOOST_CHECK_EQUAL(table.At<type>(#type)[2], F##type.data[2]);

    ICV_BASIC_TYPES_TEMPLATE(_ICV_CHECK_TEST_TABLE_DATA)
    _ICV_CHECK_TEST_TABLE_DATA(IndexType)

    // const check
    const icv::data::icvPrimitiveTableData& ctable = table;
    #define _ICV_CHECK_CONST_TEST_TABLE_DATA(type)                      \
        BOOST_CHECK_EQUAL(ctable.At<type>(#type)[0], F##type.data[0]); \
        BOOST_CHECK_EQUAL(ctable.At<type>(#type)[1], F##type.data[1]); \
        BOOST_CHECK_EQUAL(ctable.At<type>(#type)[2], F##type.data[2]);

    ICV_BASIC_TYPES_TEMPLATE(_ICV_CHECK_CONST_TEST_TABLE_DATA)
    _ICV_CHECK_CONST_TEST_TABLE_DATA(IndexType)
}

BOOST_FIXTURE_TEST_CASE(Resize, Fixture)
{
    fill();
    table.Resize(2);
    BOOST_CHECK_EQUAL(table.GetSize(), 2);
    BOOST_CHECK_EQUAL(table.At<Int8>("Int8")[0], FInt8.data[0]);
    BOOST_CHECK_EQUAL(table.At<Int8>("Int8")[1], FInt8.data[1]);
    BOOST_CHECK_EQUAL(table.At<Duration32>("Duration32")[0], FDuration32.data[0]);
    BOOST_CHECK_EQUAL(table.At<Duration32>("Duration32")[1], FDuration32.data[1]);

    table.Resize(4);
    BOOST_CHECK_EQUAL(table.GetSize(), 4);
    table.At<Int8>("Int8", 2) = FInt8.data[0];
    table.At<Int8>("Int8", 3) = FInt8.data[1];
    BOOST_CHECK_EQUAL(table.At<Int8>("Int8")[2], FInt8.data[0]);
    BOOST_CHECK_EQUAL(table.At<Int8>("Int8")[3], FInt8.data[1]);
    table.At<Duration32>("Duration32", 2) = FDuration32.data[0];
    table.At<Duration32>("Duration32", 3) = FDuration32.data[1];
    BOOST_CHECK_EQUAL(table.At<Duration32>("Duration32")[2], FDuration32.data[0]);
    BOOST_CHECK_EQUAL(table.At<Duration32>("Duration32")[3], FDuration32.data[1]);
}

BOOST_FIXTURE_TEST_CASE(copy, Fixture)
{
    fill();
    uint32_t expect_memsize = table.GetActualMemorySize();

    icv::data::icvPrimitiveTableData* copy = static_cast<icv::data::icvPrimitiveTableData*>(table.DeepCopy());
    BOOST_CHECK_EQUAL(copy->GetSize(), table.GetSize());
    BOOST_CHECK_EQUAL(copy->GetActualMemorySize(), expect_memsize);

    // TODO: Use iterator to check all the data
    BOOST_CHECK_EQUAL(table.At<Int8>("Int8")[0], copy->At<Int8>("Int8")[0]);
    BOOST_CHECK_EQUAL(table.At<Duration32>("Duration32")[0], copy->At<Duration32>("Duration32")[0]);

    copy->Dispose();
    BOOST_CHECK_EQUAL(copy->GetActualMemorySize(), 0);
    BOOST_CHECK_EQUAL(table.GetActualMemorySize(), expect_memsize);

    delete copy;
}

BOOST_AUTO_TEST_CASE(array)
{
    typedef boost::array<Int8, 5> ArrType;
    icv::data::icvPrimitiveTableData table(1, { makeTableField<ArrType>("array") });
    table.Reserve();

    ArrType indata = { 0, 1, 2, 3, 4 };
    table.At<ArrType>("array", 0) = indata;

    ArrType outdata = table.At<ArrType>("array")[0];
    BOOST_CHECK_EQUAL_COLLECTIONS(indata.begin(), indata.end(), outdata.begin(), outdata.end());
}

BOOST_AUTO_TEST_SUITE_END()
