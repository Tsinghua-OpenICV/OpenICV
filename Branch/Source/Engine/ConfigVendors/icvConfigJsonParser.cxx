#include "OpenICV/Engine/icvConfigJsonParser.h"
#include "OpenICV/Engine/icvConfigBoostHelper.h"
#include "OpenICV/Core/icvMetaData.h"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace fs = boost::filesystem;
namespace pt = boost::property_tree;

namespace icv { namespace engine
{
    icv_shared_ptr<icvMetaData> icvConfigJsonParser::Load(const fs::path& file)
    {
        pt::ptree container;
        icvMetaData::Ptr data = icv_make_shared<icvMetaData>();
        pt::read_json(file.string(), container);
        *data << container;
        return data;
    }

    void icvConfigJsonParser::Save(const fs::path& file, icv_shared_ptr<icvMetaData> content)
    {
        return;
    }
}}
