#ifndef icvConfigXmlParser_h
#define icvConfigXmlParser_h

#include "OpenICV/Engine/icvConfigBaseParser.h"

namespace icv { namespace engine
{
    // TODO: deal with structure difference between xml and json
    class icvConfigXmlParser : public icvConfigBaseParser
    {
    public:
        virtual boost::shared_ptr<icvMetaData> Load(const boost::filesystem::path& file) ICV_OVERRIDE;
        virtual void Save(const boost::filesystem::path& file, boost::shared_ptr<icvMetaData> content) ICV_OVERRIDE;
    };
}}

#endif // icvConfigXmlParser_h
