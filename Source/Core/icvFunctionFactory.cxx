#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvMetaData.h"

using namespace std;

namespace icv { namespace core
{
    icvFunctionRegistry::CreatorRegistry& icvFunctionRegistry::Registry()
    {
        static CreatorRegistry _registry;
        return _registry;
    }

    void icvFunctionRegistry::AddCreator(const string& type, icvFunctionRegistry::CreatorPtr creator)
    {
        CreatorRegistry& registry = Registry();
        registry[type] = creator;
    }

    vector<string> icvFunctionRegistry::FunctionTypeList()
    {
        CreatorRegistry& registry = Registry();
        vector<string> list;
        list.reserve(registry.size());
        for (auto item : registry)
            list.push_back(item.first);
        return list;
    }

    icvFunctionRegisterer::icvFunctionRegisterer(const std::string& type,
        icvFunctionRegistry::CreatorPtr creator)
    {
        icvFunctionRegistry::AddCreator(type, creator);
    }
}}
