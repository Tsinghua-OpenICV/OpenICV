#ifndef icvConfigYamlParser_h
#define icvConfigYamlParser_h

#include "OpenICV/Engine/icvConfigBaseParser.h"

namespace icv { namespace engine
{
    // TODO: find a minimal yaml parser library to implement this
    class icvConfigYamlParser : public icvConfigBaseParser
    {
    public:
        virtual boost::shared_ptr<icvMetaData> Load(const boost::filesystem::path& file) ICV_OVERRIDE;
        virtual void Save(const boost::filesystem::path& file, boost::shared_ptr<icvMetaData> content) ICV_OVERRIDE;
    };
}}

#endif // icvConfigYamlParser_h
