#ifndef icvFunctionFactory_h
#define icvFunctionFactory_h

#define ICV_CONFIG_CONTAINERS
#define ICV_CONFIG_POINTERS
#define ICV_CONFIG_FUNCTION
#include "OpenICV/Core/icvConfig.h"
#undef ICV_CONFIG_CONTAINERS
#undef ICV_CONFIG_POINTERS
#undef ICV_CONFIG_FUNCTION

#include <vector>
#include <boost/dll/alias.hpp>

namespace icv { namespace core
{
    class icvFunction;
    class icvMetaData;

    class icvFunctionRegistry
    {
    public:
        typedef icvFunction* (Creator)(icv_shared_ptr<icvMetaData> params);
        typedef icv_function<Creator> CreatorPtr;
        typedef icv_map<std::string, CreatorPtr> CreatorRegistry;

        static CreatorRegistry& Registry();

        // Adds a creator.
        static void AddCreator(const std::string& type, CreatorPtr creator);

        // Get a icvFunction using a icvMetaData.
        static icvFunction* CreateFunction(const std::string& type, icv_shared_ptr<icvMetaData> function_params)
        {
            return Registry()[type](function_params);
        }

        static std::vector<std::string> FunctionTypeList();

    private:
        // Static class
        icvFunctionRegistry();
    };

    class icvFunctionRegisterer {
    public:
        icvFunctionRegisterer(const std::string& type, icvFunctionRegistry::CreatorPtr creator);
    };
}}

// Use this macros to statically register function.
#define ICV_REGISTER_FUNCTION(type)                                               \
namespace _generated                                                              \
{                                                                                 \
    ::icv::core::icvFunction* _Creator_##type(                                    \
        icv_shared_ptr<::icv::core::icvMetaData> params)                          \
    {                                                                             \
        return new type(params);                                                  \
    }                                                                             \
    static ::icv::core::icvFunctionRegisterer _reg_##type(#type, _Creator_##type);\
    BOOST_DLL_ALIAS(_Creator_##type, _Export_##type)                              \
}

#endif // icvFunctionFactory_h
