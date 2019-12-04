#ifndef icvAssemblyLoader_h
#define icvAssemblyLoader_h

#include <vector>
#include <boost/filesystem/path.hpp>
#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvObject.h"

namespace icv { namespace engine
{
    class icvAssemblyLoader
    {
    public:
        static bool LoadFunction(const std::string& function);
        static bool LoadFunction(const std::vector<boost::filesystem::path>& assemblies, const std::string& function);

        static bool LoadNode(const std::string& node);
        static bool LoadNode(const std::vector<boost::filesystem::path>& assemblies, const std::string& node);

    private:
        const static std::vector<boost::filesystem::path> GetLoadDirectories(); // current path, %ICV_PATH%
        const static std::vector<boost::filesystem::path> GetLoadAssemblies();

    private:
        icvAssemblyLoader();
    };
}}

#endif // icvAssemblyLoader_h