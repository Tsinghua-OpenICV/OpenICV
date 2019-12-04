#include "OpenICV/Core/icvNode.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Core/icvDataObject.h"

namespace icv { namespace core
{
    icvNodeInput::icvNodeInput(icvNode* owner,
        icv_shared_ptr<const icvMetaData> info)
        : _owner(owner)
    {
        // TODO: implement metadata assignment
    }
   icvNodeInput::icvNodeInput(icvNode* owner)
        : _owner(owner)
    {
        // TODO: implement metadata assignment
    }

    icvDataObject* icvNodeInput::RequireDataObject()
    {
        if (_connections.size() == 0)
            return ICV_NULLPTR;
        if (_connections.size() == 1)
        {
            switch (_connections[0]->GetBufferStrategy())
            {
            case icvNodeOutputBufferStrategy::Copied:
            case icvNodeOutputBufferStrategy::BufferCopied:
           if(_connections[0]->ReadDataObject())
           {
               if(_first_copy)
               { 
                   _dataCopy=_connections[0]->ReadDataObject()->DeepCopy();_first_copy=false;
               }

                else
                {
                    // ICV_LOG_INFO<<"in FLAG"<<_connections[0]->get_flag();
                //     ICV_LOG_INFO<<"in TIME"<<_connections[0]->ReadDataObject()->GetSourceTime();

                    
                    _connections[0]->ReadDataObject()->CopyTo(_dataCopy);}
           }

                return _dataCopy ;
            case icvNodeOutputBufferStrategy::Direct:
            case icvNodeOutputBufferStrategy::Buffered:
               // _connections[0]->ReadDataObject();
                return _connections[0]->ReadDataObject();

            // TODO: Implement input queue
            default:
                break;
            }
        }
        //TODO: Process multiple connection
        return ICV_NULLPTR;
    }

    void icvNodeInput::ReleaseDataObject()
    {
        if (_connections.size() == 1)
        {
            switch (_connections[0]->GetBufferStrategy())
            {
            case icvNodeOutputBufferStrategy::Direct:
            case icvNodeOutputBufferStrategy::BufferCopied:
                _connections[0]->ReleaseDataObject();
            default:
                break;
            }
        }
        //TODO: Process multiple connection
    }

    bool icvNodeInput::CheckUpdate()
    {
        bool update = _update;
        _update = false;
        return update;
    }
}}
