#include "OpenICV/Core/icvNode.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Core/icvNodeInput.h"
#include <malloc.h>
namespace icv { namespace core
{
    icvNodeOutput::icvNodeOutput(icvNode* owner,
        icv_shared_ptr<const icvMetaData> info)
        : _owner(owner)
    {
        // TODO: implement metadata assignment
    }
  icvNodeOutput::icvNodeOutput(icvNode* owner)
        : _owner(owner)
    {
        // TODO: implement metadata assignment
    }
    icvDataObject* icvNodeOutput::RequireDataObject()
    {

        return _data;
    }


    icvDataObject* icvNodeOutput::WriteDataObject()
    {
       switch (bufferflag_)
       {
       case  0:
           return buffer_1;
        break;
       case  1:
           return buffer_2;   
        break;    
        case  2:
           return buffer_3;     
           break;    
        default:
           return _data;
           break;
       }

    }
        int icvNodeOutput::get_flag()
        {

            return bufferflag_;
        }

  icvDataObject* icvNodeOutput::ReadDataObject()
    {

    //    switch (bufferflag_)
    //    {
    //    case  0:
    //        if(_data&&buffer_input) _data->CopyTo(buffer_input);
    //     break;
    //    case  1:
    //        if(buffer_1&&buffer_input) buffer_1->CopyTo(buffer_input);   
    //     break;    
    //     case  2:
    //        if(buffer_2&&buffer_input) buffer_2->CopyTo(buffer_input);     
    //        break;    
    //     default:
    //       if(buffer_3&&buffer_input)  buffer_3->CopyTo(buffer_input);
    //        break;
            
    //    }
    // return buffer_input;

//ICV_LOG_INFO<<"OUT FLAG"<<bufferflag_;
       switch (bufferflag_)
       {
       case  0:
           return _data;
        break;
       case  1:
            return buffer_1;
        break;    
        case  2:
           return buffer_2;     
        break;    
        default:
           return buffer_3;     
           break;
            
       }


    }
    void icvNodeOutput::flag_update()
    {

       switch (bufferflag_)
       {
       case  0:
        bufferflag_=1;
        break;
       case  1:
        bufferflag_=2;
        break;    
        case  2:
        bufferflag_=3;
           break;    
        default:
        bufferflag_=0;
           break;
       }

//ICV_LOG_INFO<<"OUT FLAG"<<bufferflag_;




    }


    icvDataObject* icvNodeOutput::RequireDataObjectDeep(bool lock)
    {
        if (lock)
        {
           // _dataMutex.lock();
            // ICV_LOG_TRACE << "DataObject locked @ Output port " << this
            //     << " by Thread " << icv_this_thread::get_id();

        }
        _dataMutex.lock();
       // memcpy(buffer_1,_data,datasize_);
        _dataMutex.unlock();

       return buffer_1;
    }
    void icvNodeOutput::SetDataObject(icvDataObject* data)
    {
        // FIXME: Is this lock essential?
        //_dataMutex.lock();
        _data = data->DeepCopy();


        buffer_1=_data->DeepCopy();
       // ICV_LOG_INFO<<"TEST2";

        buffer_2=_data->DeepCopy();
        buffer_3=_data->DeepCopy();
        //buffer_input=_data->DeepCopy();

    }

    void icvNodeOutput::ReleaseDataObject()
    {
        //_dataMutex.unlock();
       // delete buffer_;
        // ICV_LOG_TRACE << "DataObject unlocked @ Output port " << this
        //     << " by Thread " << icv_this_thread::get_id();
        //free(buffer_1) ;
       // free(buffer_2) ;
        //free(buffer_3) ;
        //free(buffer_1) ;

    }

    void icvNodeOutput::Trigger()
    {
        for(icvNodeInput* connect : _connections)
            connect->_owner->Trigger(this);
    }
}}
