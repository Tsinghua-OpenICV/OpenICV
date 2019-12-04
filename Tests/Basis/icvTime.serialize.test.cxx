#include "OpenICV/Basis/icvTime.serialize.h"

#include <boost/test/unit_test.hpp>
#include <sstream>

using namespace icv;
using namespace std;
using namespace icv_chrono;

BOOST_AUTO_TEST_SUITE(icvTime_serialize)

BOOST_AUTO_TEST_CASE(serialize)
{
    SyncClock::sync(1s);

    stringstream ss;
    string data;
    Time32 t32 = Time32(11s);
    Time64 t64 = Time64(icv_chrono::duration_cast<Duration64>(11.11s));
    
    msgpack::pack(ss, t32);
    data = ss.str();
    BOOST_CHECK_EQUAL(data.length(), 6); 
    BOOST_CHECK(t32 == msgpack::unpack(data.data(), data.length()).get().as<Time32>());

    ss.clear(); ss.str("");
    msgpack::pack(ss, t64);
    data = ss.str();
    BOOST_CHECK_EQUAL(data.length(), 10);
    BOOST_CHECK(t64 == msgpack::unpack(data.data(), data.length()).get().as<Time64>());
}

BOOST_AUTO_TEST_SUITE_END()
