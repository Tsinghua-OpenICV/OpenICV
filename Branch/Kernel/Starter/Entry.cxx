#include <iostream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "OpenICV/Engine/icvNamedEngine.h"
#include "OpenICV/Engine/icvDeclare.h" // declare functions and nodes
#include "OpenICV/Core/icvZmq.h"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
using namespace std;

int main(int argc, char *argv[])
{
    po::options_description desc("Allowed options");  //名称空间+类+对象
    desc.add_options()
        ("help", "show help message")
        ("config", po::value<string>(), "load node structure from given configuration file")
        ;

    po::positional_options_description config_file; 
    config_file.add("config", 1);
    po::variables_map vm;//名称空间+类+对象

    po::store(po::command_line_parser(argc, argv).options(desc).positional(config_file).run(), vm);
    po::notify(vm);

    if (vm.count("help")) cout << desc << endl;

    if (vm.count("config"))
    {

        fs::path tpath = fs::system_complete(vm["config"].as<string>());
    
        icv::engine::icvNamedEngine engine;  //实例化对象

        engine.LoadConfiguration(tpath);  //载入配置文件

        engine.Start();
        engine.Hold();
    }
}
