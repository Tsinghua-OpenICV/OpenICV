#ifndef icvConfigJsonParser_h
#define icvConfigJsonParser_h

#include "OpenICV/Engine/icvConfigBaseParser.h"

namespace icv { namespace engine
{
    class icvConfigJsonParser : public icvConfigBaseParser
    {
    public:
        virtual icv_shared_ptr<icvMetaData> Load(const boost::filesystem::path& file) ICV_OVERRIDE;
        virtual void Save(const boost::filesystem::path& file, icv_shared_ptr<icvMetaData> content) ICV_OVERRIDE;
    };
}}

#endif // icvConfigJsonParser_h
