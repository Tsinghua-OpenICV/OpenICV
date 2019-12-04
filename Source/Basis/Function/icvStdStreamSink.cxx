#include "OpenICV/Basis/icvStdStreamSink.h"
#include "OpenICV/Core/icvDataObject.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeOutput.h"

#include <iostream>

namespace icv { namespace function
{
    ICV_REGISTER_FUNCTION(icvStdStreamSink)

    ICV_CONSTEXPR char KEY_OUTPUT_STREAM[] = "stream";

    icvStdStreamSink::icvStdStreamSink() : icvStdStreamSink(ICV_NULLPTR) {}
    icvStdStreamSink::icvStdStreamSink(icv_shared_ptr<icvMetaData> params)
        : icvFunction(1, 0, params)
    {
        if (_information.Contains(KEY_OUTPUT_STREAM))
        {
            auto stream = _information.GetString(KEY_OUTPUT_STREAM);
            if (stream == "cout") _output = &std::cout;
            else if (stream == "cerr") _output = &std::cerr;
            else if (stream == "clog") _output = &std::clog;
        }
        else _output = &std::cout;
    }
    
    // void icvStdStreamSink::ConfigurateInput(std::vector<icvNodeInput*>& inputPorts)
    // {
    //     if (!inputPorts[0]) ICV_THROW_MESSAGE("At least one input is needed!");
    // }

    void icvStdStreamSink::Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts)
    {
        icvDataObject** inData = new icvDataObject*[inputPorts.size()];
        for (int i = 0; i < inputPorts.size(); i++)
        {
            inData[i] = inputPorts[i]->RequireDataObject();
        }
        icvDataObject** outData = new icvDataObject*[outputPorts.size()];
        for (int i = 0; i < outputPorts.size(); i++)
        {
            outData[i] = outputPorts[i]->WriteDataObject();
        }
        
        *_output << inData[0]->Print() << std::endl;
    }
}}
