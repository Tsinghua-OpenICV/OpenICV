#include "OpenICV/Engine/icvNamedEngine.h"
#include "OpenICV/Engine/icvConfigDefinitions.h"
#include "OpenICV/Engine/icvAssemblyLoader.h"
#include "OpenICV/Core/icvNode.h"
#include "OpenICV/Core/icvMacros.h"
#include "OpenICV/Core/icvMetaData.h"
#include "OpenICV/Core/icvNodeFactory.h"
#include "OpenICV/Core/icvFunctionFactory.h"

#include <boost/filesystem/convenience.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;
namespace fs = boost::filesystem;

namespace icv { namespace engine
{
    icvFunction* icvNamedEngine::ParseFunction(
        icv_shared_ptr<const icvMetaData> params)
    {
        if (params->IsString(_keys::functionKey))
        {
            string function = params->GetString(_keys::functionKey);

            icvFunctionRegistry::CreatorRegistry& registry = icvFunctionRegistry::Registry();
            if (registry.count(function) || icvAssemblyLoader::LoadFunction(function))
                return icvFunctionRegistry::CreateFunction(function, icvMetaData::Ptr());
            else
                ICV_THROW_MESSAGE("Cannot find the function " + function + " in assemblies");
        }
        else if (params->IsMap(_keys::functionKey))
        {
            auto function_params = icv_make_shared<icvMetaData>(params->GetMap(_keys::functionKey));
            string function = function_params->GetString(_keys::typeKey);
            function_params->Remove(_keys::typeKey);

            icvFunctionRegistry::CreatorRegistry& registry = icvFunctionRegistry::Registry();
            if (registry.count(function) || icvAssemblyLoader::LoadFunction(function))
                return icvFunctionRegistry::CreateFunction(function, function_params);
            else
                ICV_THROW_MESSAGE("Cannot find the function " + function + " in assemblies");
        }
        else ICV_THROW_MESSAGE("Cannot find function name");
    }

    vector<icv_shared_ptr<const icvMetaData>> icvNamedEngine::ParsePorts(
        icv_shared_ptr<const icvMetaData> container_params,
        const string& node_name,
        const bool& is_input,
        vector<pair<string, string>>& connections,
        vector<string>& port_names)
    { 
        std::vector<icv_shared_ptr<const icvMetaData>> params;
        string port_key = is_input ? _keys::inputKey : _keys::outputKey;
        if (container_params->IsString(port_key))
        {
            if (is_input)
            {
                // By default, input string defines a connection
                connections.push_back(pair<string, string>(
                    node_name + _keys::portSplitChar + "0", container_params->GetString(port_key)));
                port_names.push_back("");
                params.push_back(icv_make_shared<icvMetaData>());
            }
            else
            {
                // By default, output string defines a port name
                port_names.push_back(container_params->GetString(port_key));
                params.push_back(icv_make_shared<icvMetaData>());
            }
        }
        else if (container_params->IsArray(port_key))
        {

            auto port_array = container_params->GetArray(port_key);
            for (size_t port = 0; port < port_array.Size(); port++)
            {
                if (port_array.IsString(port))
                {


                    if (is_input)
                    {
                        // By default, input string defines a connection
                        connections.push_back(pair<string, string>(
                        node_name + _keys::portSplitChar + to_string(port), port_array.GetString(port)));
                        params.push_back(icv_make_shared<icvMetaData>());
                    }
                    else
                    {
                        // By default, output string defines a port name
                        /////////////////////////////////////////////////////////

                        port_names.push_back(port_array.GetString(port));
                        params.push_back(icv_make_shared<icvMetaData>());

                        /////////////////////////////////////////////////////////
                    }
                }
                else if (port_array.IsMap(port))
                {
                    auto port_params = icv_make_shared<icvMetaData>(port_array.GetMap(port));

                    // Parse name
                    if (port_params->Contains(_keys::nameKey))
                    {
                        port_names.push_back(port_params->GetString(_keys::nameKey));
                        port_params->Remove(_keys::nameKey);
                    }
                    else port_names.push_back("");

                    // Parse connection
                    if (port_params->Contains(_keys::connectKey))
                    {

                        if(port_params->IsString(_keys::connectKey))
                        {
                  ///////////////////////////////////////////////////////////////////////////////////////////   
       
                            if (is_input)
                            {
                              //  connections.push_back(pair<string, string>(
                             //       node_name + _keys::portSplitChar + to_string(port), port_params->GetString(_keys::connectKey)));
                                     connections.push_back(pair<string, string>(
                                    node_name + _keys::portSplitpoint + port_names.back(), port_params->GetString(_keys::connectKey)));
                            }
                            else
                            {
                                connections.push_back(pair<string, string>(
                                    port_params->GetString(_keys::connectKey), node_name + _keys::portSplitpoint + port_names.back()));
                            }

                  ///////////////////////////////////////////////////////////////////////////////////////////          
                        }
                        else if(port_params->IsArray(_keys::connectKey))
                        {
                            auto connection_array = port_params->GetArray(_keys::connectKey);
                            for (size_t conn = 0; conn < connection_array.Size(); conn++)
                            {
                                if (is_input)
                                {
                                    connections.push_back(pair<string, string>(
                                        node_name + _keys::portSplitpoint + port_names.back(), port_params->GetString(_keys::connectKey)));
                                }
                                else
                                {
                                    connections.push_back(pair<string, string>(
                                        port_params->GetString(_keys::connectKey), node_name + _keys::portSplitpoint + port_names.back()));
                                }
                            }
                        }
                        else ICV_THROW_MESSAGE("Unknown port connection definition");
                        port_params->Remove(_keys::connectKey);
                    }

                    params.push_back(port_params);
                }
                else ICV_THROW_MESSAGE("Unknown port configuration type at port " + port);
            }
        }
        else ICV_THROW_MESSAGE("Unknown port configuration type");

        return params;
    }

    void icvNamedEngine::LoadConfiguration(fs::path configFile)
    {
        icvMetaData::Ptr info;

        // Read configuration content from files
        string ext = fs::extension(configFile);
        if (ext == ".ini")
        {
            // TODO: To be implemented
        }
        else if (ext == ".json")
        {
            icvConfigJsonParser parser;
            info = parser.Load(configFile);
        }
        else if (ext == ".yaml")
        {
            // TODO: To be implemented
        }
        else if (ext == ".xml")
        {
            // TODO: To be implemented
        }
        else ICV_THROW_MESSAGE("Unsupported configuration file type!");

        vector<pair<string, string>> connections;
        for (auto key : info->GetKeys())
        {

            // Copy params for node initialization
            auto node_params = icv_make_shared<icvMetaData>(info->GetMap(key));
            if (node_params->Contains(_keys::typeKey))
            {
                auto node_type = node_params->GetString(_keys::typeKey);
                node_params->Remove(_keys::typeKey);
                icvFunction* function = ICV_NULLPTR;

                             // Parse input and output ports
                vector<string> input_names, output_names;
                vector<icv_shared_ptr<const icvMetaData>> input_params, output_params;
                if(node_params->Contains(_keys::inputKey))
                {
                    input_params = ParsePorts(node_params, key, true, connections, input_names);
                    ICV_LOG_INFO<<"size of input"<<input_params.size();
                    node_params->Remove(_keys::inputKey);
                }
                if(node_params->Contains(_keys::outputKey))
                {
                    output_params = ParsePorts(node_params, key, false, connections, output_names);
                    node_params->Remove(_keys::outputKey);
                    ICV_LOG_INFO<<"size of output"<<output_params.size();

                }


                // Parse function and remove function key
                if(node_params->Contains(_keys::functionKey))
                {
                    function = ParseFunction(node_params);
                    node_params->Remove(_keys::functionKey);
                }

   
                // Construct node
                auto node = icv_shared_ptr<icvNode>(icvNodeRegistry::CreateNode(node_type,
                    function, node_params, input_params.data(), output_params.data(),input_names.size(),output_names.size()));

                // Register ports
                for (size_t port = 0; port < input_names.size(); port++)
                    if (!input_names[port].empty())
                        _input_name_map.emplace(key+_keys::portSplitpoint+input_names[port], node->GetInputPort(port));
                for (size_t port = 0; port < output_names.size(); port++)
                    if (!output_names[port].empty())
                        _output_name_map.emplace(key+_keys::portSplitpoint+output_names[port], node->GetOutputPort(port));

                // Register node
                _node_name_map.emplace(key, node);
                _nodes.emplace(node);
            }
            else ICV_THROW_MESSAGE("Cannot find the node type of node " + key);
        }

        // Add connections
    //     for (auto connection_pair : connections)
    //     {
    //         string input_str = connection_pair.first, output_str = connection_pair.second;
    //         auto input_split = input_str.rfind(_keys::portSplitChar);
    //         auto output_split = output_str.rfind(_keys::portSplitChar);

    //         int input_port_idx = input_split != string::npos ? boost::lexical_cast<int>(input_str.substr(input_split + 1)) : 0;
    //         auto input = _node_name_map[input_str.substr(0, input_split)]->GetInputPort(input_port_idx);
    //         int output_port_idx = output_split != string::npos ? boost::lexical_cast<int>(output_str.substr(output_split + 1)) : 0;
    //         auto output = _node_name_map[output_str.substr(0, output_split)]->GetOutputPort(output_port_idx);
            
    //         Connect(input, output);
    //     }
        for (auto connection_pair : connections)
        {
            string input_str = connection_pair.first, output_str = connection_pair.second;
                                            ICV_LOG_INFO<<input_str<<":"<<output_str;

            auto input = _input_name_map[input_str];
            auto output = _output_name_map[output_str];
            
            Connect(input, output);
        }


     }

    void icvNamedEngine::SaveConfiguration(boost::filesystem::path configFile)
    {
        // TODO: not implemented yet
    }
}}
