#include "OpenICV/Basis/icvMetaData.serialize.h"

#include <boost/test/unit_test.hpp>
#include <sstream>

using namespace icv::core;
using namespace std;

BOOST_AUTO_TEST_SUITE(icvMetaData_serialize)

BOOST_AUTO_TEST_CASE(serialize)
{
    icv::core::icvMetaData map;
    stringstream ss;

    map.SetString("string", "Test1");
    map.SetInteger("integer", 1);
    map.AddArray("array");
    map.GetArray("array").AddString("Test2");
    map.GetArray("array").AddInteger(2);
    map.GetArray("array").AddMap();
    map.GetArray("array").GetMap(2).SetString("string", "Test3");
    map.GetArray("array").GetMap(2).SetInteger("integer", 3);

    msgpack::pack(ss, map);
    string data = ss.str();
    BOOST_CHECK_EQUAL(data.length(), 63);

    auto obj = msgpack::unpack(data.data(), data.length());
    icv::core::icvMetaData copy = obj.get().as<icv::core::icvMetaData>();
    BOOST_CHECK_EQUAL(map.GetArray("array").GetMap(2).GetString("string"), 
                      copy.GetArray("array").GetMap(2).GetString("string"));
    BOOST_CHECK(map == copy);
}

BOOST_AUTO_TEST_SUITE_END()
