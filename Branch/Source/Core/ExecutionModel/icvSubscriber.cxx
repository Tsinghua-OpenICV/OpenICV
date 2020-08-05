#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvNode.h"
#include <malloc.h>
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Core/icvDataObject.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"

namespace icv { namespace core
{
    icvSubscriber::icvSubscriber(icvFunction* owner,
        icv_shared_ptr<const icvMetaData> info)
        : _owner(owner)
    {
        // TODO: implement metadata assignment
    }
   icvSubscriber::icvSubscriber(icvFunction* owner)
        : _owner(owner)
    {
        // TODO: implement metadata assignment
    }
   bool icvSubscriber::isconnected()
    {
        if(GetConnections().size()>0)return true;
        else  return false;

    }

       void icvSubscriber::enable_Trigger(){

           _Triggermode=true;
       }
       void icvSubscriber::disable_Trigger(){

           _Triggermode=false;

       }
        bool icvSubscriber::is_Triggered (){
            return _Triggermode;
        }

  void icvSubscriber::setname(string name){

         to_pub_name=name;
       }
  string icvSubscriber::getname(){

         return to_pub_name;
       }


    icvDataObject* icvSubscriber::RequireDataObject()
    {

        if (_connections.size() == 0)
            return ICV_NULLPTR;
        if (_connections.size() == 1)
        {
            switch (_connections.at(0)->GetBufferStrategy())
            {
            case icvPublisherBufferStrategy::Copied:

            case icvPublisherBufferStrategy::Shm:

            case icvPublisherBufferStrategy::BufferCopied:

           if(_connections[0]->ReadDataObject())
           {
               if(_first_copy)
               { 
                   _dataCopy=_connections[0]->ReadDataObject()->DeepCopy();
                   _first_copy=false;
               }

                else
                {

                    _connections[0]->ReadDataObject()->CopyTo(_dataCopy);

                 }
                                 return _dataCopy ;

            }
            else return ICV_NULLPTR;
           
            case icvPublisherBufferStrategy::Direct:
            case icvPublisherBufferStrategy::PointerCopy:
                return _connections[0]->ReadDataObject();

            // TODO: Implement input queue
            default:
                break;
            }
        }
        //TODO: Process multiple connection
        return ICV_NULLPTR;
    }

    void icvSubscriber::ReleaseDataObject()
    {
        if (_connections.size() == 1)
        {
            switch (_connections[0]->GetBufferStrategy())
            {
            case icvPublisherBufferStrategy::Direct:
            case icvPublisherBufferStrategy::BufferCopied:
                _connections[0]->ReleaseDataObject();
            default:
                break;
            }
        }
        //TODO: Process multiple connection
    }

    bool icvSubscriber::CheckUpdate()
    {
        bool update = _update;
        _update = false;
        return update;
    }
}}
