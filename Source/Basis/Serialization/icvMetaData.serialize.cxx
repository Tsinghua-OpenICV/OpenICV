#include "OpenICV/Basis/icvMetaData.serialize.h"

using namespace std;
using namespace icv;
using namespace icv::core;

namespace msgpack
{
    MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
    {
        namespace adaptor
        {
            msgpack::object const& convert<icvMetaData>::operator()
                (msgpack::object const& o, icvMetaData& v) const
            {
                if(o.type!= type::MAP) throw msgpack::type_error();
                for (int i = 0; i < o.via.map.size; i++)
                {
                    string key = o.via.map.ptr[i].key.as<string>();
                    msgpack::v2::object& val = o.via.map.ptr[i].val;
                    switch (val.type)
                    {
                    case type::STR:
                        v.SetString(key, val.as<string>());
                        break;
                    case type::ARRAY:
                        v.AddArray(key);
                        val.convert(v.GetArray(key));
                        break;
                    case type::MAP:
                        v.AddMap(key);
                        val.convert(v.GetMap(key));
                        break;
                    }
                }
                return o;
            }

            msgpack::object const& convert<icvMetaDataArray>::operator()
                (msgpack::object const& o, icvMetaDataArray& v) const
            {
                if (o.type != type::ARRAY) throw msgpack::type_error();
                for (int i = 0; i < o.via.array.size; i++)
                {
                    msgpack::v2::object& val = o.via.array.ptr[i];
                    switch (val.type)
                    {
                    case type::STR:
                        v.AddString(val.as<string>());
                        break;
                    case type::ARRAY:
                        v.AddArray();
                        val.convert(v.GetArray(v.Size() - 1));
                        break;
                    case type::MAP:
                        v.AddMap();
                        val.convert(v.GetMap(v.Size() - 1));
                        break;
                    }
                }
                return o;
            }
        }
    } // MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
} // namespace msgpack