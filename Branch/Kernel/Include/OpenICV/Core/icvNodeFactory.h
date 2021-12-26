#ifndef icvNodeFactory_h
#define icvNodeFactory_h

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
    class icvNode;
    class icvFunction;
    class icvMetaData;

    class icvNodeRegistry
    {
    public:
        typedef icvNode* Creator(icvFunction* function,
            icv_shared_ptr<const icvMetaData> params,
            icv_shared_ptr<const icvMetaData>* input_params,
            icv_shared_ptr<const icvMetaData>* output_params);
        typedef icv_function<Creator> CreatorPtr;
        typedef icv_map<std::string, CreatorPtr> CreatorRegistry;

        static CreatorRegistry& Registry();

        // Adds a creator.
        static void AddCreator(const std::string& type, CreatorPtr creator);

        // Get a icvNode using a icvMetaData.
        static icvNode* CreateNode(const std::string& type, icvFunction* function,
            icv_shared_ptr<const icvMetaData> params,
            icv_shared_ptr<const icvMetaData>* input_params,
            icv_shared_ptr<const icvMetaData>* output_params)
        {
            return Registry()[type](function, params, input_params, output_params);
        }

        static std::vector<std::string> NodeTypeList();

    private:
        // Static class
        icvNodeRegistry();
    };

    class icvNodeRegisterer {
    public:
        icvNodeRegisterer(const std::string& type, icvNodeRegistry::CreatorPtr creator);
    };
}}

// Use this macros to statically register node.
#define ICV_REGISTER_NODE(type)                                                   \
namespace _generated                                                              \
{                                                                                 \
    ::icv::core::icvNode* _Creator_##type(::icv::core::icvFunction* function,     \
        icv_shared_ptr<const icvMetaData> params,                                 \
        icv_shared_ptr<const icvMetaData>* input_params,                          \
        icv_shared_ptr<const icvMetaData>* output_params)                         \
    {                                                                             \
        return new type(function, params, input_params, output_params);           \
    }                                                                             \
    static ::icv::core::icvNodeRegisterer _reg_##type(#type, _Creator_##type);    \
    BOOST_DLL_ALIAS(_Creator_##type, _Export_##type)                              \
}

#endif // icvNodeFactory_h
