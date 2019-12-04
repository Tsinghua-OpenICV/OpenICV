#ifndef icvConfigBaseParser_h
#define icvConfigBaseParser_h

#define ICV_CONFIG_POINTERS
#include <OpenICV/Core/icvConfig.h>
#undef ICV_CONFIG_POINTERS

#include <boost/filesystem/path.hpp>

#include "OpenICV/Core/icvObject.h"
#include "OpenICV/Core/icvMetaData.h"

namespace icv { namespace engine
{
    using namespace icv::core;

    class icvConfigBaseParser : public icvObject
    {
    public:
        virtual icv_shared_ptr<icvMetaData> Load(const boost::filesystem::path& file) = 0;
        virtual void Save(const boost::filesystem::path& file, icv_shared_ptr<icvMetaData> content) = 0;
    };
}}

#endif // icvConfigBaseParser_h
