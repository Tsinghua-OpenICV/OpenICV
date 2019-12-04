#ifndef icvConfigIniParser_h
#define icvConfigIniParser_h

#include "OpenICV/Engine/icvConfigBaseParser.h"

namespace icv { namespace engine
{
    // TODO: deal with repeated key in ini files which is not supported by boost
    class icvConfigIniParser : public icvConfigBaseParser
    {
    public:
        virtual icv_shared_ptr<icvMetaData> Load(const boost::filesystem::path& file) ICV_OVERRIDE;
        virtual void Save(const boost::filesystem::path& file, icv_shared_ptr<icvMetaData> content) ICV_OVERRIDE;
    };
}}

#endif // icvConfigIniParser_h
