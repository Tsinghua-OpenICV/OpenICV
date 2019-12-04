#ifndef icvMetaData_serialize_h
#define icvMetaData_serialize_h

#include "OpenICV/Core/icvMetaData.h"

#include <boost/variant/get.hpp>
#include <msgpack.hpp>

namespace msgpack
{
    MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
    {
        namespace adaptor
        {
            template<> struct convert<icv::core::icvMetaData>
            {
                msgpack::object const& operator()(msgpack::object const& o, icv::core::icvMetaData& v) const;
            };
            template<> struct convert<icv::core::icvMetaDataArray>
            {
                msgpack::object const& operator()(msgpack::object const& o, icv::core::icvMetaDataArray& v) const;
            };

            template<> struct pack<icv::core::icvMetaData>
            {
                template <typename Stream>
                packer<Stream>& operator()(msgpack::packer<Stream>& o, icv::core::icvMetaData const& v) const
                {
                    o.pack_map(v.Size());
                    auto keys = v.GetKeys();
                    for (auto iter = keys.begin(); iter != keys.end(); iter++)
                    {
                        o.pack(*iter);
                        const icv::core::icvMetaDataElementType& elem = v.Get(*iter);
                        switch (elem.which())
                        {
                        case icv::core::icvMetaDataElementString:
                            o.pack(boost::get<std::string>(elem));
                            break;
                        case icv::core::icvMetaDataElementMap:
                            o.pack(*boost::get<icv::core::icvMetaData*>(elem));
                            break;
                        case icv::core::icvMetaDataElementArray:
                            o.pack(*boost::get<icv::core::icvMetaDataArray*>(elem));
                            break;
                        }
                    }
                    return o;
                }
            };
            template<> struct pack<icv::core::icvMetaDataArray>
            {
                template <typename Stream>
                packer<Stream>& operator()(msgpack::packer<Stream>& o, icv::core::icvMetaDataArray const& v) const
                {
                    o.pack_array(v.Size());
                    for (int idx = 0; idx < v.Size(); idx++)
                    {
                        const icv::core::icvMetaDataElementType& elem = v.Get(idx);
                        switch (elem.which())
                        {
                        case icv::core::icvMetaDataElementString:
                            o.pack(boost::get<std::string>(elem));
                            break;
                        case icv::core::icvMetaDataElementMap:
                            o.pack(*boost::get<icv::core::icvMetaData*>(elem));
                            break;
                        case icv::core::icvMetaDataElementArray:
                            o.pack(*boost::get<icv::core::icvMetaDataArray*>(elem));
                            break;
                        }
                    }
                    return o;
                }
            };
        } // namespace adaptor
    } // MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
} // namespace msgpack

#endif
