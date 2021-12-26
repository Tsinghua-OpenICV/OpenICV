#ifndef icvNamedEngined_h
#define icvNamedEngined_h

#define ICV_CONFIG_CONTAINERS
#define ICV_CONFIG_POINTERS
#include "OpenICV/Core/icvConfig.h"
#undef ICV_CONFIG_CONTAINERS
#undef ICV_CONFIG_POINTERS
 
#include <boost/filesystem/path.hpp>

#include "OpenICV/Engine/icvEngine.h"

namespace icv { namespace engine
{
    class icvNamedEngine : public icvEngine
    {
    public:
        // Load node states from file
        void LoadConfiguration(boost::filesystem::path configFile);
        void SaveConfiguration(boost::filesystem::path configFile);

    protected:
        icv_map<std::string, icv_shared_ptr<icvNode>>* _node_name_map=new icv_map<std::string, icv_shared_ptr<icvNode>>();
        icv_map<std::string, icvSubscriberInterface*> _input_name_map;
        icv_map<std::string, icvPublisherInterface*> _output_name_map;
    
    private:
        icvFunction* ParseFunction(icv_shared_ptr<const icvMetaData> container_params);

        std::vector<icv_shared_ptr<const icvMetaData>> ParsePorts(
            icv_shared_ptr<const icvMetaData> container_params, // in params
            const std::string& node_name,
            const bool& is_input,
            std::vector<std::pair<std::string, std::string>>& connections, // out params
            std::vector<std::string>& port_names);
    };
}}

#endif // icvNamedEngined_h
