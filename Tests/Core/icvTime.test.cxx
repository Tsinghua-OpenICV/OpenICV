#include "OpenICV/Core/icvTime.h"

#include <boost/test/unit_test.hpp>
#include <type_traits>
#include <sstream>

BOOST_AUTO_TEST_SUITE(icvTime)

using namespace icv;

BOOST_AUTO_TEST_CASE(size)
{
    BOOST_CHECK(std::is_pod<Duration32>::value);
    BOOST_CHECK(std::is_pod<Duration64>::value);

    BOOST_CHECK_EQUAL(sizeof(Duration32), 4);
    BOOST_CHECK_EQUAL(sizeof(Duration64), 8);
}

BOOST_AUTO_TEST_CASE(numeric_literal)
{
    // BOOST_CHECK_EQUAL needs `<<` implemented for chrono types
    BOOST_CHECK(1s == icv_chrono::seconds(1));
    BOOST_CHECK(1ns == icv_chrono::nanoseconds(1));
    BOOST_CHECK(1.002s == icv_chrono::nanoseconds(1002000000));
}

BOOST_AUTO_TEST_CASE(shift_operator)
{
    std::stringstream ss;
    ss << Duration32(1s);
    BOOST_CHECK_EQUAL(ss.str(), std::string("1s"));

    ss.str(""); ss.clear();
    ss << 1000ns;
    BOOST_CHECK_EQUAL(ss.str(), std::string("0.000001s"));
}

BOOST_AUTO_TEST_SUITE_END()
