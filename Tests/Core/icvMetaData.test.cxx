#include "OpenICV/Core/icvMetaData.h"

#include <boost/test/unit_test.hpp>

using namespace icv::core;

BOOST_AUTO_TEST_SUITE(icvMetaData)

BOOST_AUTO_TEST_CASE(assign)
{
    icv::core::icvMetaData data;

    data.SetBoolean("boolean", true);
    data.SetDecimal("decimal", -1.23456);
    data.SetInteger("integer", -123456);
    data.SetString("string", "Test");

    BOOST_CHECK_EQUAL(data.GetBoolean("boolean"), true);
    // BOOST_CHECK_CLOSE
    BOOST_CHECK_EQUAL(data.GetDecimal("decimal"), -1.23456);
    BOOST_CHECK_EQUAL(data.GetInteger("integer"), -123456);
    BOOST_CHECK_EQUAL(data.GetString("string"), "Test");

    BOOST_CHECK(data.IsBoolean("boolean"));
    BOOST_CHECK(data.IsDecimal("decimal"));
    BOOST_CHECK(data.IsInteger("integer"));
    BOOST_CHECK(data.IsString("string"));
}

BOOST_AUTO_TEST_CASE(properties)
{
    icv::core::icvMetaData data;
    BOOST_CHECK_EQUAL(data.Size(), 0);

    data.SetBoolean("boolean", true);
    data.SetDecimal("decimal", -1.23456);
    data.SetInteger("integer", -123456);
    data.SetString("string", "Test");

    BOOST_CHECK_EQUAL(data.Size(), 4);
    auto keys = data.GetKeys();
    BOOST_CHECK_EQUAL(keys.size(), 4);
    BOOST_CHECK(keys.find("boolean") != keys.end());
    BOOST_CHECK(keys.find("decimal") != keys.end());
    BOOST_CHECK(keys.find("integer") != keys.end());
    BOOST_CHECK(keys.find("string") != keys.end());

    data.Clear();
    BOOST_CHECK_EQUAL(data.Size(), 0);
}

BOOST_AUTO_TEST_SUITE_END()


BOOST_AUTO_TEST_SUITE(icvMetaDataArray)

BOOST_AUTO_TEST_CASE(assign)
{
    icv::core::icvMetaDataArray data;
    
    data.AddBoolean(true);
    data.AddDecimal(-1.23456);
    data.AddInteger(-123456);
    data.AddString("Test");

    BOOST_CHECK_EQUAL(data.GetBoolean(0), true);
    // BOOST_CHECK_CLOSE
    BOOST_CHECK_EQUAL(data.GetDecimal(1), -1.23456);
    BOOST_CHECK_EQUAL(data.GetInteger(2), -123456);
    BOOST_CHECK_EQUAL(data.GetString(3), "Test");

    BOOST_CHECK(data.IsBoolean(0));
    BOOST_CHECK(data.IsDecimal(1));
    BOOST_CHECK(data.IsInteger(2));
    BOOST_CHECK(data.IsString(3));
}

BOOST_AUTO_TEST_CASE(properties)
{
    icv::core::icvMetaDataArray data;
    BOOST_CHECK_EQUAL(data.Size(), 0);

    data.AddBoolean(true);
    data.AddDecimal(-1.23456);
    data.AddInteger(-123456);
    data.AddString("Test");

    BOOST_CHECK_EQUAL(data.Size(), 4);

    data.Clear();
    BOOST_CHECK_EQUAL(data.Size(), 0);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(icvMetaDataNested)

BOOST_AUTO_TEST_CASE(construct_and_copy)
{
    icv::core::icvMetaData map;

    map.SetString("string", "Test1");
    map.AddArray("array");
    map.GetArray("array").AddString("Test2");
    map.GetArray("array").AddMap();
    map.GetArray("array").GetMap(1).SetString("string", "Test3");

    icv::core::icvMetaData copy = map;

    BOOST_CHECK_EQUAL(map.GetString("string"), copy.GetString("string"));
    BOOST_CHECK_EQUAL(map.GetArray("array").GetString(0), copy.GetArray("array").GetString(0));
    BOOST_CHECK_EQUAL(map.GetArray("array").GetMap(1).GetString("string"),
                      copy.GetArray("array").GetMap(1).GetString("string"));

    copy.GetArray("array").GetMap(1).SetString("string", "Test4");
    BOOST_CHECK_EQUAL(map.GetString("string"), copy.GetString("string"));
    BOOST_CHECK_EQUAL(map.GetArray("array").GetString(0), copy.GetArray("array").GetString(0));
    BOOST_CHECK_NE(map.GetArray("array").GetMap(1).GetString("string"),
                   copy.GetArray("array").GetMap(1).GetString("string"));
}

BOOST_AUTO_TEST_CASE(compare)
{
    icv::core::icvMetaData map;

    map.SetString("string", "Test1");
    map.AddArray("array");
    map.GetArray("array").AddString("Test2");
    map.GetArray("array").AddMap();
    map.GetArray("array").GetMap(1).SetString("string", "Test3");

    icv::core::icvMetaData copy = map;

    BOOST_CHECK(map.GetArray("array").GetMap(1) == copy.GetArray("array").GetMap(1));
    BOOST_CHECK(map.GetArray("array") == copy.GetArray("array"));
    BOOST_CHECK(map == copy);

    copy.GetArray("array").GetMap(1).SetString("string", "Test4");
    BOOST_CHECK(map.GetArray("array").GetMap(1) != copy.GetArray("array").GetMap(1));
    BOOST_CHECK(map.GetArray("array") != copy.GetArray("array"));
    BOOST_CHECK(map != copy);
}

BOOST_AUTO_TEST_SUITE_END()
