#include "OpenICV/Core/icvNode.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Core/icvSubscriber.h"
#include <malloc.h>
namespace icv { namespace core
{
    icvPublisher::icvPublisher(icvFunction* owner,
        icv_shared_ptr<const icvMetaData> info)
        : _owner(owner)
    {
        // TODO: implement metadata assignment
    }
  icvPublisher::icvPublisher(icvFunction* owner)
        : _owner(owner)
    {
        // TODO: implement metadata assignment
    }
    icvDataObject* icvPublisher::RequireDataObject()
    {

        return ReadDataObject();
    }
void icvPublisher::set_send_data(icvDataObject* data_to_send)
     {
        //send_Output<icv::data::icvInt64Data >(port);

		
  if(data_to_send)
      {  
		data_to_send_=data_to_send;
     }

   
       };
void icvPublisher::Send_Out()
     {
        //send_Output<icv::data::icvInt64Data >(port);

		if(data_to_send_)
        {
		_lock.lock();
       icvDataObject* pub_buff= WriteDataObject();
      
      if(pub_buff==ICV_NULLPTR)
      {SetDataObject(data_to_send_);
        pub_buff= WriteDataObject();
      }
            
        data_to_send_->CopyTo(pub_buff);
        flag_update();

        _lock.unlock();

        Trigger();
        }


       };


    icvDataObject* icvPublisher::WriteDataObject()
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
        int icvPublisher::get_flag()
        {

            return bufferflag_;
        }

  icvDataObject* icvPublisher::ReadDataObject()
    {

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
    void icvPublisher::flag_update()
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


    icvDataObject* icvPublisher::RequireDataObjectDeep(bool lock)
    {
        if (lock)
        {
           // _lock.lock();
            // ICV_LOG_TRACE << "DataObject locked @ Output port " << this
            //     << " by Thread " << icv_this_thread::get_id();

        }
        _lock.lock();
       // memcpy(buffer_1,_data,datasize_);
        _lock.unlock();

       return buffer_1;
    }
    void icvPublisher::SetDataObject(icvDataObject* data)
    {
        // FIXME: Is this lock essential?
        //_lock.lock();
        _data = data->DeepCopy();


        buffer_1=_data->DeepCopy();
       // ICV_LOG_INFO<<"TEST2";

        buffer_2=_data->DeepCopy();
        buffer_3=_data->DeepCopy();
        //buffer_input=_data->DeepCopy();


        

    }

    void icvPublisher::ReleaseDataObject()
    {
        //_lock.unlock();
       // delete buffer_;
        // ICV_LOG_TRACE << "DataObject unlocked @ Output port " << this
        //     << " by Thread " << icv_this_thread::get_id();
        //free(buffer_1) ;
       // free(buffer_2) ;
        //free(buffer_3) ;
        //free(buffer_1) ;

    }

    void icvPublisher::Trigger()
    {
        for(icvSubscriber* connect : _connections)
        {  
            if(connect->is_Triggered())            
            connect->_owner->get_node()->Trigger(this);

        }
    }
}}
