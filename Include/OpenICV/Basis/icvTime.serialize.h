#ifndef icvMetaData_serialize_h
#define icvMetaData_serialize_h

#include "OpenICV/Core/icvTime.h"

#include <msgpack.hpp>

namespace msgpack
{
    MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
    {
        namespace adaptor
        {
            template<> struct convert<icv::Time32>
            {
                msgpack::object const& operator()(msgpack::object const& o, icv::Time32& v) const;
            };
            template<> struct convert<icv::Time64>
            {
                msgpack::object const& operator()(msgpack::object const& o, icv::Time64& v) const;
            };

            // All time stamp are serialized under SyncClock
            template<> struct pack<icv::Time32>
            {
                template <typename Stream>
                packer<Stream>& operator()(msgpack::packer<Stream>& o, icv::Time32 const& v) const
                {
                    icv::Time32 stime = v +
                        icv_chrono::duration_cast<icv::Duration32>(icv::SyncClock::offset());
                    std::uint32_t sec = stime.time_since_epoch().count();

                    o.pack_ext(4, -1);
                    char data[4]; _msgpack_store32(data, sec);
                    o.pack_ext_body(data, 4);
                    return o;
                }
            };
            template<> struct pack<icv::Time64>
            {
                template <typename Stream>
                packer<Stream>& operator()(msgpack::packer<Stream>& o, icv::Time64 const& v) const
                {
                    icv::Time64 stime = v + icv::SyncClock::offset();
                    std::uint64_t sec = stime.time_since_epoch().count() / 1000000000;
                    std::uint64_t nsec = stime.time_since_epoch().count() % 1000000000;

                    std::uint64_t data64 = (nsec << 34) | sec;
                    o.pack_ext(8, -1);
                    char data[8]; _msgpack_store64(data, data64);
                    o.pack_ext_body(data, 8);
                    return o;
                }
            };

            template <> struct object_with_zone<icv::Time32>
            {
                void operator()(msgpack::object::with_zone& o, icv::Time32 const& v) const;
            };
            template <> struct object_with_zone<icv::Time64>
            {
                void operator()(msgpack::object::with_zone& o, icv::Time64 const& v) const;
            };
        } // namespace adaptor
    } // MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
} // namespace msgpack

#endif
