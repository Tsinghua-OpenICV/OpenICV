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

      

       if( params->IsMap(_keys::functionKey))
       {
       // string nodename_t=params->GetString(_keys::nodename);

        auto function_params = icv_make_shared<icvMetaData>(params->GetMap(_keys::functionKey));
        auto function = function_params->GetString(_keys::nameKey);
        function_params->Remove(_keys::nameKey);
        function_params->AddMap(_keys::nodename);
        function_params->SetString(_keys::nodename,params->GetString(_keys::nodename));
        icvFunctionRegistry::CreatorRegistry& registry = icvFunctionRegistry::Registry();
        //ICV_LOG_INFO<<"LOAD node "+nodename_t+" load function"+function;

        if (registry.count(function) || icvAssemblyLoader::LoadFunction(function)){
                return icvFunctionRegistry::CreateFunction(function, function_params);}
         else{
                //ICV_THROW_MESSAGE("Cannot find the function " + function + " in assemblies");
         }
       }
       // ICV_LOG_INFO<<"icvNamedEngine Construction finished";
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
   
         if (container_params->IsArray(port_key))
        {

            auto port_array = container_params->GetArray(port_key);
            for (size_t port = 0; port < port_array.Size(); port++)
            {
              
                 if (port_array.IsMap(port))
                {
                    auto port_params = icv_make_shared<icvMetaData>(port_array.GetMap(port));

                    // Parse name
                    if (port_params->Contains(_keys::nameKey))
                    {
                        port_names.push_back(port_params->GetString(_keys::nameKey));
                        port_params->Remove(_keys::nameKey);
                    }
                    else 
                    
                    {
                        port_names.push_back(""+port);
                        if (!is_input) ICV_THROW_MESSAGE(node_name+" publisher should have a name!");
                    }
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
                                    node_name + _keys::portSplitpoint + port_params->GetString(_keys::connectKey), port_params->GetString(_keys::connectKey)));
                            }
                            else
                            {
                                connections.push_back(pair<string, string>(
                                    port_params->GetString(_keys::connectKey),  port_names.back()));
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
                                        node_name + _keys::portSplitpoint + port_params->GetString(_keys::connectKey), port_params->GetString(_keys::connectKey)));
                                }
                                else
                                {
                                    connections.push_back(pair<string, string>(
                                        port_params->GetString(_keys::connectKey), port_names.back()));
                                }
                            }
                        }
                        else ICV_THROW_MESSAGE("Unknown port connection definition");
                        port_params->Remove(_keys::connectKey);
                    }

                    params.push_back(port_params);
                }
                else ICV_THROW_MESSAGE("Unknown port configuration type at "+ port);
            }
        }
        else ICV_THROW_MESSAGE("Unknown "+port_key+" configuration type");

        return params;
    }

    void icvNamedEngine::LoadConfiguration(fs::path configFile)
    {

            context_= zmq_ctx_new();    
        //ICV_LOG_INFO<<"first contexx_"<<context_;
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
            node_params->AddMap(_keys::nodename);
            node_params->SetString(_keys::nodename,key);
            string node_type;
            if (node_params->Contains(_keys::typeKey))
            {
                 node_type= node_params->GetString(_keys::typeKey);
                node_params->Remove(_keys::typeKey);
            }
            else  node_type="icvThreadedNode";
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

                //ICV_LOG_INFO<<"Loading file";

                // Parse function and remove function key

                function = ParseFunction(node_params);
                //ICV_LOG_INFO<<"Parsing function1";

                node_params->Remove(_keys::functionKey);
                //ICV_LOG_INFO<<"Parsing function2";

                function->set_zmq_context_uniq(context_);
                //ICV_LOG_INFO<<"Parsing function3";

                function->addconnections(&connections);
                //ICV_LOG_INFO<<"Parsing function4";

                // Construct node
                auto node = icv_shared_ptr<icvNode>(icvNodeRegistry::CreateNode(node_type,
                    function, node_params, input_params.data(), output_params.data()));


             //  icv_map< int,std::string> * temp_map=function->get_pubsub_map();
              vector<std::string> *temp_sub_name=function->get_sub_names();
              vector<std::string> *temp_pub_name=function->get_pub_names();
                // Register ports
              // ICV_LOG_INFO<<"INFO TEMP pub "<<temp_pub_name->size();

                for (auto subname_temp: *temp_sub_name)
                   { 

                    if (!subname_temp.empty())
                        _input_name_map.emplace(key+_keys::portSplitpoint+subname_temp, function->GetInputPort(subname_temp));
                    //_input_name_map.emplace(subname_temp, function->GetInputPort(subname_temp));

                   }
                for (auto pubname_temp: *temp_pub_name)
                    {
                        if (!pubname_temp.empty())
                        _output_name_map.emplace(pubname_temp, function->GetOutputPort(pubname_temp));
                        
                    }

                // Register node
                _node_name_map->emplace(key, node);
                _nodes.emplace(node);

            }
            //std::cout<<"Registering node number: "<<_node_name_map->size();
            node_manager=new icvNodeManager(_node_name_map);
        //for (auto it:_input_name_map){cout<<"input name map: "<<it.first<<endl;}
        //for (auto it:_output_name_map){cout<<"output name map: "<<it.first<<endl;}

         
        for (auto connection_pair : connections)
        {
            string input_str = connection_pair.first, output_str = connection_pair.second;
            //ICV_LOG_INFO<<input_str<<" : "<<output_str;

            auto input = _input_name_map[input_str];
            if (_output_name_map.find(output_str)!=_output_name_map.end()){
          
            auto output = _output_name_map[output_str];
            Connect(input, output);
            }
            else
            {  
                ICV_LOG_INFO<<"No corresponding publisher to subscriber "<<input_str;
            }
            
        }

       //ICV_LOG_INFO<<"Load Configuration file Finished";
     }

    void icvNamedEngine::SaveConfiguration(boost::filesystem::path configFile)
    {
        // TODO: not implemented yet
    }
}}
