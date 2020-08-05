#include <iostream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "OpenICV/Engine/icvNamedEngine.h"
#include "OpenICV/icvDeclare.h" // declare functions and nodes

namespace po = boost::program_options;
namespace fs = boost::filesystem;
using namespace std;

int main(int argc, char *argv[])
{
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "show help message")
        ("config", po::value<string>(), "load node structure from given configuration file")
        ;

    po::positional_options_description config_file;
    config_file.add("config", 1);
    po::variables_map vm;

    po::store(po::command_line_parser(argc, argv).options(desc).positional(config_file).run(), vm);
    po::notify(vm);

    if (vm.count("help")) cout << desc << endl;

    if (vm.count("config"))
    {

        fs::path tpath = fs::system_complete(vm["config"].as<string>());
        icv::engine::icvNamedEngine engine;

        engine.LoadConfiguration(tpath);

        engine.Start();
        engine.Hold();
    }
}
