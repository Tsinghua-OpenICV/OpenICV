#include "OpenICV/Core/icvMacros.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeFactory.h"
#include "OpenICV/Engine/icvAssemblyLoader.h"

#include <boost/predef.h>
#include <boost/algorithm/string.hpp>
#include <boost/dll/import.hpp>
#include <boost/filesystem/convenience.hpp>

using namespace std;
namespace dll = boost::dll;
namespace fs = boost::filesystem;
using fs::path;
using namespace icv::core;

namespace icv { namespace engine
{
    const vector<path> icvAssemblyLoader::GetLoadDirectories()
    {
#ifdef BOOST_OS_WINDOWS
        ICV_CONSTEXPR auto separator = ";";
#else
        ICV_CONSTEXPR auto separator = ":";
#endif
        vector<path> paths, path_separated;
        paths.push_back(fs::system_complete(".")); // Add current directory

        // Add ICV_PATH environment to the list
        if (getenv("ICV_PATH"))
        {
            path_separated.clear();
            const string env_icv_path(getenv("ICV_PATH"));
            split(path_separated, env_icv_path, boost::is_any_of(separator), boost::token_compress_on);
            for (auto path : path_separated) paths.push_back(path);
        }

        return paths;
    }

    const vector<path> icvAssemblyLoader::GetLoadAssemblies()
    {
#if BOOST_OS_WINDOWS==BOOST_VERSION_NUMBER_AVAILABLE
        ICV_CONSTEXPR auto dllext = ".dll";
#elif BOOST_OS_MACOS
        ICV_CONSTEXPR auto dllext = ".dylib";
#else // BOOST_OS_LINUX
        ICV_CONSTEXPR auto dllext = ".so";
#endif

        vector<path> list;
        for (auto path : icvAssemblyLoader::GetLoadDirectories())
        {
            for (auto &fpath : fs::directory_iterator(path))
            {
                if (fs::is_regular_file(fpath.path()))
                {
        //        ICV_LOG_TRACE<<"file path:"<<fpath;

                    string ext = fs::extension(fpath.path());
                    if (ext == dllext) list.push_back(fpath.path());
                }
            }
        }
        return list;
    }

    bool icvAssemblyLoader::LoadFunction(const std::string& function)
    {
        return LoadFunction(GetLoadAssemblies(), function);
    }

    bool icvAssemblyLoader::LoadFunction(const vector<path>& assemblies, const std::string& function)
    {

        //ICV_LOG_TRACE<<"assemble path:"<<assemblies.size();

        for (size_t i = 0; i < assemblies.size(); i++)
        {

             ICV_LOG_INFO<<"assemble path:"<<assemblies[i];

            auto lib = new dll::shared_library(assemblies[i]);
            if (lib->has("_Export_" + function))
            {
                icvFunctionRegistry::CreatorPtr creator = dll::import_alias<icvFunctionRegistry::Creator>(*lib, "_Export_" + function);
                icvFunctionRegistry::AddCreator(function, creator);
                return true;
            }
        }
        return false;
    }

    bool icvAssemblyLoader::LoadNode(const std::string& node)
    {
        return LoadNode(GetLoadAssemblies(), node);
    }

    bool icvAssemblyLoader::LoadNode(const vector<path>& assemblies, const std::string& node)
    {
        for (size_t i = 0; i < assemblies.size(); i++)
        {
            auto lib = new dll::shared_library(assemblies[i]);
            if (lib->has("_Export_" + node))
            {
                icvNodeRegistry::CreatorPtr creator = dll::import_alias<icvNodeRegistry::Creator>(*lib, "_Export_" + node);
                icvNodeRegistry::AddCreator(node, creator);
                return true;
            }
        }
        return false;
    }
}}
