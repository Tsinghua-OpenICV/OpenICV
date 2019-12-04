#include "OpenICV/Basis/icvTime.serialize.h"

using namespace std;
using namespace icv;
using namespace icv_chrono;

namespace msgpack
{
    MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
    {
        namespace adaptor
        {
            msgpack::object const& convert<Time32>::operator()
                (msgpack::object const& o, Time32& v) const
            {
                if (o.type != type::EXT) throw msgpack::type_error();
                if (o.via.ext.type() != -1) { throw msgpack::type_error(); }
                if (o.via.ext.size != 4) { throw msgpack::type_error(); }

                uint32_t sec; _msgpack_load32(uint32_t, o.via.ext.data(), &sec);

                v = Time32(Duration32(sec) - duration_cast<icv::Duration32>(SyncClock::offset()));
                return o;
            }

            msgpack::object const& convert<Time64>::operator()
                (msgpack::object const& o, Time64& v) const
            {
                if (o.type != type::EXT) throw msgpack::type_error();
                if (o.via.ext.type() != -1) { throw msgpack::type_error(); }
                if (o.via.ext.size != 8) { throw msgpack::type_error(); }

                uint64_t data64; _msgpack_load64(uint64_t, o.via.ext.data(), &data64);
                uint64_t nsec = data64 >> 34;
                uint64_t sec = data64 & 0x00000003ffffffffL;

                v = Time64(Duration64(sec * 1000000000 + nsec) - SyncClock::offset());
                return o;
            }

            void object_with_zone<icv::Time32>::operator()
                (msgpack::object::with_zone& o, icv::Time32 const& v) const
            {
                throw "Not implemented";
            }
            void object_with_zone<icv::Time64>::operator()
                (msgpack::object::with_zone& o, icv::Time64 const& v) const
            {
                throw "Not implemented";
            }
        }
    } // MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
} // namespace msgpack