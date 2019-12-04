#include "OpenICV/Basis/icvRecordNode.h"
#include "OpenICV/Core/icvDataObject.h"
#include "OpenICV/Core/icvNodeFactory.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Basis/icvMsgpackRecorder.h"

#define ICV_CONFIG_THREAD
#define ICV_CONFIG_FUNCTION
#include "OpenICV/Core/icvConfig.h"
#undef ICV_CONFIG_THREAD
#undef ICV_CONFIG_FUNCTION

#include <iostream>
#include <algorithm>
#include <boost/filesystem/convenience.hpp>

namespace icv { namespace node
{
    ICV_REGISTER_NODE(icvRecordNode)

    ICV_CONSTEXPR char KEY_OUTPUT_FILEPATH[] = "path";

    icvRecordNode::icvRecordNode(icvFunction* function,
        icv_shared_ptr<const icvMetaData> params,
        icv_shared_ptr<const icvMetaData>* input_params,
        icv_shared_ptr<const icvMetaData>* output_params,int num_inport,int num_outport)
        : icvNode(ICV_NULLPTR, params, input_params, output_params, num_inport, num_outport)
    {
        std::string path;
        if (_information.Contains(KEY_OUTPUT_FILEPATH))
            path = _information.GetString(KEY_OUTPUT_FILEPATH);
        else path = std::to_string(DefaultClock::now().time_since_epoch().count());

        // FIXME: use msgpack recorder as default currently, should be read from configuration
        _recorder = new _impl::icvMsgpackRecorder(boost::filesystem::system_complete(path), 0);
    }

    void icvRecordNode::Start(icvMonitor* monitor)
    {
        _recorder->Play();
        for(Uint32 port = 0; port < _outputPorts.size(); port++)
            icv_thread loop(icv_bind(&icvRecordNode::InnerLoop, this, port));
    }

    void icvRecordNode::Progress()
    {
        return; // TODO: implement
    }

    void icvRecordNode::Abort()
    {
        return; // TODO: implement
    }

    void icvRecordNode::Trigger(icvNodeOutput* caller)
    {
        for (int ip = 0; ip < _inputPorts.size(); ip++)
        {
            auto pos = std::find(_inputPorts[ip]->GetConnections().begin(),
                _inputPorts[ip]->GetConnections().end(), caller);
            if (pos != _inputPorts[ip]->GetConnections().end())
            {
                _recorder->Record(caller->RequireDataObject(), ip);
                caller->ReleaseDataObject();
            }
        }
    }

    void icvRecordNode::InnerLoop(Uint32 port)
    {
        while (true) 
        {
            _recorder->PlayNext(_outputPorts[port]->RequireDataObject(), port);
            _outputPorts[port]->ReleaseDataObject();
        }
    }
}}
