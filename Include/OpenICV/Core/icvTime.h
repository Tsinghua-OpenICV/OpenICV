#ifndef icvTime_h
#define icvTime_h

#define ICV_CONFIG_THREAD
#define ICV_CONFIG_TIME
#include "OpenICV/Core/icvConfig.h"
#undef ICV_CONFIG_TIME
#undef ICV_CONFIG_THREAD

#include <iostream>


namespace icv
{
    typedef icv_chrono::duration<std::int32_t, icv_ratio<1>> Duration32;
     typedef icv_chrono::duration<std::int32_t, icv_ratio<1,1000>> Duration32ms;
    typedef icv_chrono::duration<std::int64_t, icv_ratio<1,1000000>> Duration64us;
    typedef icv_chrono::duration<std::int64_t, icv_ratio<1, 1000000000>> Duration64;
    typedef icv_chrono::system_clock DefaultClock;
    typedef icv_chrono::time_point<DefaultClock, Duration32> Time32;
    typedef icv_chrono::time_point<DefaultClock, Duration64> Time64;
    // TODO: support Time96 as said in https://www.boost.org/doc/libs/1_67_0/doc/html/chrono/users_guide.html#chrono.users_guide.examples.clocks.xtime_clock

    // represent a clock synchronized over devices
    struct SyncClock
    {
        typedef icv_chrono::system_clock base_clock;
        typedef Duration64 duration;
        //typedef Duration64::rep rep;
        //typedef Duration64::period period;
        typedef icv_chrono::time_point<SyncClock> time_point;
                typedef icv_chrono::time_point<SyncClock,Duration32> time_point_s;
         typedef icv_chrono::time_point<SyncClock,Duration32ms> time_point_ms;
    typedef icv_chrono::time_point<SyncClock,Duration64us> time_point_us;
        
        static ICV_CONSTEXPR_FUNC bool is_steady() { return false; }
        static bool is_synced();
        static void sync(const duration& offset);

        static time_point now();
    static time_point_s now_s();
	static time_point_ms now_ms();
	static time_point_us now_us();
	static time_point now_ns();
        static time_t time_s();
	static time_t time_ms();
	static time_t time_us();
	static time_t time_ns();
        static const duration& offset();
    };
}

#if !defined(BOOST_NO_CXX11_USER_DEFINED_LITERALS)
#if defined(BOOST_NO_CXX11_HDR_CHRONO) || defined(BOOST_NO_CXX14_BINARY_LITERALS) || defined(OPENICV_PREFER_BOOST)

// // convenient duration expressions are left in header for inline compilation
// ICV_CONSTEXPR_FUNC icv::Duration32 operator "" s(unsigned long long value)
// {
//     return icv::Duration32(value);
// }

// ICV_CONSTEXPR_FUNC icv::Duration64 operator "" s(long double value)
// {
//     // add 0.5 to round the double
//     return icv::Duration64(static_cast<unsigned long long>(value * 1000000000 + 0.5));
// }

// ICV_CONSTEXPR_FUNC icv::Duration64 operator "" ns(unsigned long long value)
// {
//     return icv::Duration64(value);
// }
#else

using namespace std::chrono_literals;

#endif
#endif

namespace std // wrap in std namespace to make boost recognize these
{
    std::string to_string(icv::Duration32 value);
    std::string to_string(icv::Duration64 value);
    std::ostream& operator <<(std::ostream&, const icv::Duration32&);
    std::ostream& operator <<(std::ostream&, const icv::Duration64&);
    std::istream& operator >>(std::istream&, icv::Duration32&);
    std::istream& operator >>(std::istream&, icv::Duration64&);
}

#endif
