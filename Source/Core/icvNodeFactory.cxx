#include "OpenICV/Core/icvNodeFactory.h"
#include "OpenICV/Core/icvMetaData.h"

using namespace std;

namespace icv { namespace core
{
    icvNodeRegistry::CreatorRegistry& icvNodeRegistry::Registry()
    {
        static CreatorRegistry _registry;
        return _registry;
    }

    void icvNodeRegistry::AddCreator(const string& type, icvNodeRegistry::CreatorPtr creator)
    {
        CreatorRegistry& registry = Registry();
        registry[type] = creator;
    }

    vector<string> icvNodeRegistry::NodeTypeList()
    {
        CreatorRegistry& registry = Registry();
        vector<string> list;
        list.reserve(registry.size());
        for (auto item : registry)
            list.push_back(item.first);
        return list;
    }

    icvNodeRegisterer::icvNodeRegisterer(const std::string& type, icvNodeRegistry::CreatorPtr creator)
    {
        icvNodeRegistry::AddCreator(type, creator);
    }
}}
