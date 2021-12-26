#ifndef icvPublisher_h
#define icvPublisher_h

#define ICV_CONFIG_TYPEINDEX
#define ICV_CONFIG_THREAD
#define ICV_CONFIG_ATOMIC

#include "OpenICV/Core/icvConfig.h"
#undef ICV_CONFIG_TYPEINDEX
#undef ICV_CONFIG_THREAD
#undef ICV_CONFIG_ATOMIC


#include "OpenICV/Core/icvObject.h"
#include "OpenICV/Core/icvMacros.h"
#include "OpenICV/Core/icvDataObject.h"

// #include "OpenICV/Basis/icvStructureData.hxx"
// #include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvNode.h"
#include "OpenICV/Core/icvSubscriber.h"

#include <malloc.h>
#include <sstream>

namespace icv { namespace core
{
    using namespace std;
    // class icvDataObject;
    class icvNode;
    class icvSubscriberInterface;
    template <typename T, typename>
    class icvSubscriber;
    class icvFunction;

    ICV_ENUM_CLASS icvPublisherFrequency
    {
        Lazy, // Sample frequency is larger than data change frequency
        Instant // Sample frequency is smaller than data change frequency
    };

    ICV_ENUM_CLASS icvPublisherBufferStrategy
    {
        Direct, // Output data buffer is locked when execution and then disposed
        Copied, // Output data buffer is copied before execution and then disposed
        /* Above strategies are forbiddened if the output have multiple connections */
        Buffered, // Output data buffer is locked when execution
        PointerCopy,//transfer pointer
        BufferCopied, // Output data buffer is copied before execution
        Queued, // Output data are queued and processed one by one
        Shm
    };

    class icvPublisherInterface {
        public:
        virtual std::vector<icvSubscriberInterface*>& GetConnections()=0;
        virtual icv_mutex& GetDataMutex()=0;
        virtual icvPublisherBufferStrategy GetBufferStrategy()=0;
        virtual icvPublisherFrequency GetFrequency()=0;
        virtual std::string GetSerializedBuffer()=0;
        virtual void SetBufferStrategy(const icvPublisherBufferStrategy&)=0;
        virtual void SetFrequency(const icvPublisherFrequency&)=0;


        virtual icvDataObject* RequireDataObject()=0;
        virtual icvDataObject* RequireDataObjectDeep(bool=true) =0;
        virtual void set_send_data(icvDataObject*) =0;
        virtual void Send_Out() =0;
        virtual void SetDataObject(icvDataObject*) =0;
        virtual void ReleaseDataObject() =0; // Release the lock of data object
        virtual void writedata() =0;
        virtual void Trigger() =0;
        virtual icvDataObject* WriteDataObject() =0;
        virtual icvDataObject* ReadDataObject() =0;
        virtual void flag_update() =0;
        virtual int get_flag() =0;
        virtual string getname() =0;
        virtual void setname(const string name_t) =0;
        virtual bool isSuitableDataType(icvDataObject*) =0;


    };
    template <typename T, typename = typename std::enable_if<std::is_base_of<icv::core::icvDataObject, T>::value>::type>
    // template <typename T, typename>
    class icvPublisher : public icvObject, public icvPublisherInterface
    {
    public:
        icvPublisher(icvFunction* owner, icv_shared_ptr<const icvMetaData> info): _owner(owner)
        {
            // TODO: implement metadata assignment
        }
        icvPublisher(icvFunction* owner): _owner(owner)
        {
            // TODO: implement metadata assignment
        }

        ICV_PROPERTY_GETSET(Frequency, _frequency, icvPublisherFrequency)
        ICV_PROPERTY_GETSET(BufferStrategy, _bufferStrategy, icvPublisherBufferStrategy)

        ICV_PROPERTY_GET(Connections, _connections, std::vector<icvSubscriberInterface*>&)
        ICV_PROPERTY_GET(DataMutex, _dataMutex, icv_mutex&)

        ICV_PROPERTY_GET(SerializedBuffer, _serialized_buffer, std::string)

        icvDataObject* RequireDataObject()
        {

        return ReadDataObject();
        }
        icvDataObject* RequireDataObjectDeep(bool lock = true)
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
        void set_send_data(icvDataObject* data_to_send)
        {
            // T* temp_data_ptr = dynamic_cast<T*>(data_to_send);
            if(isSuitableDataType(data_to_send))
            { 
                data_to_send_=data_to_send;
            } else {
                ICV_THROW_MESSAGE("icvPublisher::set_send_data : the type of data to send doesn't match the definition.");
            }
            // else throw type error
        }
        void Send_Out()
        {
            //send_Output<icv::data::icvInt64Data>(port);

            if(data_to_send_)
            {
            //_lock.lock();

            // icvDataObject* pub_buff= WriteDataObject();
            
            // if(pub_buff==ICV_NULLPTR)
            // {
            //     SetDataObject(data_to_send_);
            //     pub_buff= WriteDataObject();
            // }
            if(_data){data_to_send_->CopyTo(_data);}
            else {_data = data_to_send_->DeepCopy();}
                
            // flag_update();s
            updateSerializedBuf();
            //_lock.unlock();

            Trigger();
            }


        }

        void SetDataObject(icvDataObject* data)
        {
            // FIXME: Is this lock essential?
            //_lock.lock();
            // T* temp_data_ptr = dynamic_cast<T*>(data);
            if(dynamic_cast<T*>(data))
            { 
                _data = data->DeepCopy();

                buffer_1=_data->DeepCopy();
                // ICV_LOG_INFO<<"TEST2";
                buffer_2=_data->DeepCopy();
                buffer_3=_data->DeepCopy();
                //buffer_input=_data->DeepCopy();

            } else {
                ICV_LOG_WARN<<"icvPublisher::set_send_data : data type doesn't match";
            }
            //else throw type match error
        }        

        void ReleaseDataObject() // Release the lock of data object
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
        void writedata(){};
        void Trigger()
        {
            for(icvSubscriberInterface* connect : _connections)
            {  
                if(connect->is_Triggered())            
                // connect->_owner->get_node()->Trigger(this);
                //for temp test
                connect->GetOwner()->get_node()->Trigger(this);

            }
        }        
        icvDataObject* WriteDataObject()
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
        icvDataObject* ReadDataObject()
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
        void flag_update()
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

        int get_flag()
        {
            return bufferflag_;
        }
        string getname(){return publishername_;};
        void setname(const string name_t){ publishername_=name_t;};
        bool isSuitableDataType(icvDataObject* dataToCheck)
        {
            if(dynamic_cast<T*>(dataToCheck)) return true;
            else return false;
        };



        icvDataObject* _data = ICV_NULLPTR;
    private:
        icvFunction* _owner;
        std::vector<icvSubscriberInterface*> _connections;
        int bufferflag_=3;// the last finished writing flag
        icvPublisherFrequency _frequency;
        icvPublisherBufferStrategy _bufferStrategy = icvPublisherBufferStrategy::BufferCopied;
        
        icvDataObject* buffer_1= ICV_NULLPTR, *buffer_2= ICV_NULLPTR, *buffer_3= ICV_NULLPTR, *buffer_input= ICV_NULLPTR;
        icvDataObject* data_to_send_=ICV_NULLPTR;
        std::string _serialized_buffer;
        icv_mutex dataoutlock;
        string publishername_;

        size_t datasize_;
        icv_mutex _dataMutex;
        spin_lock _lock;

        bool _first_send=true;


        void updateSerializedBuf(){
            std::stringstream ssBuf;
            _data->Serialize(ssBuf, 0);
            // switch (bufferflag_)
            // {
            // case  0:
            //     buffer_1->Serialize(ssBuf, 0);
            //     break;    
            // case  1:
            //     buffer_2->Serialize(ssBuf, 0);     
            //     break;    
            // case  2:
            //     buffer_3->Serialize(ssBuf, 0);     
            //     break;  
            // default:
            //     _data->Serialize(ssBuf, 0);
            //     break;
            // }
            _serialized_buffer = ssBuf.str();
        }
    };

    // TODO: support multiple type variants
    // TODO: support constructor params

    template<class T> bool CheckDataType(icvPublisherInterface* output, size_t buffersize_)
    {
        if (dynamic_cast<icvPublisher<T>*>(output)) return true;
        else return false;

        // return std::is_same<typename std::decay<S>::type,T>::value;

        // if (output->RequireDataObject())
            // return dynamic_cast<S*>(output->RequireDataObject());
        // else
        // {
            // ICV_LOG_TRACE << "Instantiated" << icv_typeid(T).name() << " at output";
            // output->SetDataObject((icvDataObject*)new T);
            // return true;
            // return false;
        // }
    }
}}

#endif // icvPublisher_h