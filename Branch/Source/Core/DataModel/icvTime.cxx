#include <OpenICV/Core/icvTime.h>
#include <string>

// Instantiate time types



template class icv_chrono::duration<std::uint32_t, icv_ratio<1>>;
template class icv_chrono::duration<std::uint32_t, icv_ratio<1,1000>>;
template class icv_chrono::duration<std::uint32_t, icv_ratio<1,1000000>>;
template class icv_chrono::duration<std::uint64_t, icv_ratio<1, 1000000000>>;
template class icv_chrono::time_point<icv::DefaultClock, icv::Duration32>;
template class icv_chrono::time_point<icv::DefaultClock, icv::Duration32ms>;
template class icv_chrono::time_point<icv::DefaultClock, icv::Duration64us>;
template class icv_chrono::time_point<icv::DefaultClock, icv::Duration64>;
namespace icv
{
    static bool _synced = false;
    static icvTime::duration _offset = icvTime::duration(0);
    bool icvTime::is_synced() { return _synced; }
    void icvTime::sync(const duration& offset)
    {
        _offset = offset;
        _synced = true;
    }
    icvTime::time_point icvTime::now()
    {
        return time_point(base_clock::now().time_since_epoch() + _offset);
    }

     icvTime::time_point_s icvTime::now_s()
    {
       // base_clock::time_point systime= base_clock::now(); 
       // Time64 offset_time(_offset);
       // //Time64(_offset);
         Duration64 dura_64=base_clock::now().time_since_epoch() + _offset;
       // Time32 time_32=icv_chrono::time_point_cast<Duration32>(time_64);
        //Duration64 asd=_offset+base_clock::now().time_since_epoch();
        time_point time_64=time_point(dura_64);
        time_point_s time_32=icv_chrono::time_point_cast<Duration32>(time_64);
        return time_32;
    }
      icvTime::time_point_ms icvTime::now_ms()
    {
       // base_clock::time_point systime= base_clock::now(); 
       // Time64 offset_time(_offset);
       // //Time64(_offset);
         Duration64 dura_64=base_clock::now().time_since_epoch() + _offset;
       // Time32 time_32=icv_chrono::time_point_cast<Duration32>(time_64);
        //Duration64 asd=_offset+base_clock::now().time_since_epoch();
        time_point time_64=time_point(dura_64);
        time_point_ms time_32ms=icv_chrono::time_point_cast<Duration32ms>(time_64);
        return time_32ms;
    }
          icvTime::time_point_us icvTime::now_us()
    {
       // base_clock::time_point systime= base_clock::now(); 
       // Time64 offset_time(_offset);
       // //Time64(_offset);
         Duration64 dura_64=base_clock::now().time_since_epoch() + _offset;
       // Time32 time_32=icv_chrono::time_point_cast<Duration32>(time_64);
        //Duration64 asd=_offset+base_clock::now().time_since_epoch();
        time_point time_64=time_point(dura_64);
        time_point_us time_64us=icv_chrono::time_point_cast<Duration64us>(time_64);
        return time_64us;
    }
         icvTime::time_point icvTime::now_ns()
    {
       // base_clock::time_point systime= base_clock::now(); 
       // Time64 offset_time(_offset);
       // //Time64(_offset);
         Duration64 dura_64=base_clock::now().time_since_epoch() + _offset;
       // Time32 time_32=icv_chrono::time_point_cast<Duration32>(time_64);
        //Duration64 asd=_offset+base_clock::now().time_since_epoch();
        time_point time_64=time_point(dura_64);
        //time_point time_32=icv_chrono::time_point_cast<Duration32>(time_64);
        return time_64;
    }

           time_t icvTime::time_ns()
    {

        return now_ns().time_since_epoch().count();
    }

           time_t icvTime::time_us()
    {

        return now_us().time_since_epoch().count();
    }

           time_t icvTime::time_ms()
    {

        return now_ms().time_since_epoch().count();
    }

           time_t icvTime::time_s()
    {

        return now_s().time_since_epoch().count();
    }
    const icvTime::duration& icvTime::offset() { return _offset; }
}

namespace std
{
    std::string to_string(icv::Duration32 value)
    {
        return to_string(value.count()) + 's';
    }

    std::string to_string(icv::Duration64 value)
    {
        return to_string(icv_chrono::duration_cast<icv_chrono::duration<double>>(value).count()) + 's';
    }

    std::ostream& operator<<(std::ostream& stream, icv::Duration32 const& d)
    {
        stream << to_string(d);
        return stream;
    }

    std::ostream& operator<<(std::ostream& stream, icv::Duration64 const& d)
    {
        stream << to_string(d);
        return stream;
    }

    std::istream& operator>>(std::istream& stream, icv::Duration32 &d)
    {
        unsigned long long s;
        stream >> s;
        d = icv::Duration32(s);
        return stream;
    }

    std::istream& operator>>(std::istream& stream, icv::Duration64 &d)
    {
        double s;
        stream >> s;
        icv_chrono::duration<double> t(s);
        d = icv_chrono::duration_cast<icv::Duration64>(t);
        return stream;
    }
}
