#include "OpenICV/Core/icvNode.h"
#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Core/icvDataObject.h"

using namespace std;

namespace icv { namespace core
{
    icvNode::icvNode(icvFunction* function) : icvNode(function, ICV_NULLPTR, ICV_NULLPTR, ICV_NULLPTR,10 ,5) {}

    icvNode::icvNode(icvFunction* function,
        icv_shared_ptr<const icvMetaData> info,
        icv_shared_ptr<const icvMetaData>* input_info,
        icv_shared_ptr<const icvMetaData>* output_info,int num_inport,int num_outport) :
        _function(function)
    {
        ICV_LOG_TRACE << "Initialize Node @" << this << " :";


 _num_in=num_inport,_num_out=num_outport;
        if (function != ICV_NULLPTR)
        {
            _inputPorts = vector<icvNodeInput*>(_num_in);
            for (int i = 0; i < _num_in; i++)
            {
              //  if (input_info) _inputPorts[i] = new icvNodeInput(this, input_info[i]);
                 _inputPorts[i] = new icvNodeInput(this);
              //  ICV_LOG_TRACE << "    Input Port " << i << " @" << _inputPorts[i];
            }

             _outputPorts = vector<icvNodeOutput*>(_num_out);
            for (int i = 0; i <_num_out; i++)
            {
               // if (output_info) _outputPorts[i] = new icvNodeOutput(this, output_info[i]);
                 _outputPorts[i] = new icvNodeOutput(this);
               // ICV_LOG_TRACE << "    Output Port " << i << " @" << _outputPorts[i];
            }
        }

        if (info) _information = *info;
    }

    icvNodeInput* icvNode::GetInputPort(int port)
    {
        if (port == _inputPorts.size())
            _inputPorts.push_back(new icvNodeInput(this, ICV_NULLPTR));
        else if (port > _inputPorts.size())
        {
            _inputPorts.resize(port + 1);
            _inputPorts[port] = new icvNodeInput(this, ICV_NULLPTR);
        }

        return _inputPorts[port];
    }

    icvNodeOutput* icvNode::GetOutputPort(int port)
    {
        if (port == _outputPorts.size())
            _outputPorts.push_back(new icvNodeOutput(this, ICV_NULLPTR));
        else if (port > _outputPorts.size())
        {
            _outputPorts.resize(port + 1);
            _outputPorts[port] = new icvNodeOutput(this, ICV_NULLPTR);
        }

        return _outputPorts[port];
    }

    icvNode::~icvNode()
    {
        // FIXME: Ensures the connection are closed?
        for (int i = 0; i < _inputPorts.size(); i++)
            delete _inputPorts[i];
        for (int i = 0; i < _outputPorts.size(); i++)
            delete _outputPorts[i];
        delete _function;
    }

    void icvNode::Execute()
    {

        // Gather input and output data object
        // icvDataObject** inputs = new icvDataObject*[_inputPorts.size()];
        // for (int i = 0; i < _inputPorts.size(); i++)
        // {
        //     inputs[i] = _inputPorts[i]->RequireDataObject();
        //     inputs[i]->Reserve();
        // }
        // icvDataObject** outputs = new icvDataObject*[_outputPorts.size()];
        // for (int i = 0; i < _outputPorts.size(); i++)
        // {
        //     outputs[i] = _outputPorts[i]->RequireDataObject();
        //     outputs[i]->Reserve();
        // }

        // Execute the function
        _function->buff_Input(_inputPorts);
        _function->Execute(_inputPorts, _outputPorts);

        // Release possible locks
        for (int i = 0; i < _inputPorts.size(); i++)
        {
            _inputPorts[i]->ReleaseDataObject();
        }

        for (int i = 0; i < _outputPorts.size(); i++)
        {
            _outputPorts[i]->ReleaseDataObject();
            _outputPorts[i]->Trigger();
        }

       // ICV_LOG_INFO << "test updates @ Thread " << icv_this_thread::get_id();
    }

    void Connect(icvNode::Ptr producer, int producerPort, icvNode::Ptr consumer, int consumerPort)
    {
        Connect(consumer->GetInputPort(consumerPort), producer->GetOutputPort(producerPort));
    }

    void Disconnect(icvNode::Ptr producer, int producerPort, icvNode::Ptr consumer, int consumerPort)
    {
        Disconnect(consumer->GetInputPort(consumerPort), producer->GetOutputPort(producerPort));
    }

    void Connect(icvNodeInput* input, icvNodeOutput* output)
    {
        input->GetConnections().push_back(output);
        output->GetConnections().push_back(input);
    }
    void Disconnect(icvNodeInput* input, icvNodeOutput* output)
    {
        // Remove output port from input connections
        auto inconnection = find(input->GetConnections().begin(), input->GetConnections().end(), output);
        if (inconnection != input->GetConnections().end()) input->GetConnections().erase(inconnection);

        // Remove input port from output connections
        auto outconnection = find(output->GetConnections().begin(), output->GetConnections().end(), input);
        if (outconnection != output->GetConnections().end()) output->GetConnections().erase(outconnection);
    }
}}
