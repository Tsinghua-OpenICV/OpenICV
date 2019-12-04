#include "OpenICV/Engine/icvNamedEngine.h"
#include "OpenICV/Basis/icvThreadedNode.h"

#include "OpenICV/Basis/icvStdStreamSink.h"
#include "OpenICV/Basis/icvRandomSource.h"
#include "AddFunction.hxx"
#include "PolynomialFunction.hxx"

#include <iostream>
#include <string>
#include <boost/filesystem/operations.hpp>

using namespace std;
using namespace icv;
using namespace icv::core;
using namespace icv::node;
using namespace icv::function;
using namespace icv::engine;

int main(int argc, char *argv[])
{
    if (argc <= 1 || argc > 2)
    {
        cout << "Usage: SimplePipeline <program|ini|json|yaml|xml>" << endl
            << "       The parameter indicates how to build up the network" << endl;
        return 1;
    }

    string type(argv[1]);
    icvNamedEngine* engine = new icvNamedEngine;
    if (type == "program")
    {
        auto node = new icvRandomSource(); node->SetInterval(100);
        auto source1 = icv_make_shared<icvThreadedNode>(node);

        node = new icvRandomSource(); node->SetInterval(200);
        auto source2 = icv_make_shared<icvThreadedNode>(node);

        auto params = icv_make_shared<icvMetaData>();
        params->AddArray("coefficients");
        params->GetArray("coefficients").AddInteger(1);
        params->GetArray("coefficients").AddInteger(2);
        params->GetArray("coefficients").AddInteger(3);
        auto filter1 = icv_make_shared<icvThreadedNode>(new PolynomialFunction(params));

        auto filter2 = icv_make_shared<icvThreadedNode>(new AddFunction(params));
        auto sink = icv_make_shared<icvThreadedNode>(new icvStdStreamSink);

        Connect(source1, 0, filter1, 0);
        Connect(filter1, 0, filter2, 0);
        Connect(source2, 0, filter2, 1);
        Connect(filter2, 0, sink, 0);

        engine->AddNode(source1);
        engine->AddNode(source2);
        engine->AddNode(filter1);
        engine->AddNode(filter2);
        engine->AddNode(sink);
    }
    else if (type == "json")
    {
        boost::filesystem::path tpath = boost::filesystem::system_complete("SimplePipeline.json");
        engine->LoadConfiguration(tpath);
    }

    engine->Start();
    engine->Hold();

    delete engine;
    return 0;
}
