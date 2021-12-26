#ifndef icvSubscriber_h
#define icvSubscriber_h
#define ICV_CONFIG_TYPEINDEX
#define ICV_CONFIG_THREAD
#define ICV_CONFIG_POINTERS
#define ICV_CONFIG_FUNCTION
#include "OpenICV/Core/icvConfig.h"
#undef ICV_CONFIG_TYPEINDEX
#undef ICV_CONFIG_THREAD
#undef ICV_CONFIG_POINTERS
#undef ICV_CONFIG_FUNCTION
#include <vector>
#include "OpenICV/Core/icvObject.h"
#include "OpenICV/Core/icvMacros.h"
#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvPublisher.h"
using namespace std;
namespace icv { namespace core
{
    class icvNode;
    class icvDataObject;
    class icvPublisherInterface;
    // template <typename T, typename = typename std::enable_if<std::is_base_of<icv::core::icvDataObject, T>::value>::type>
    // template <typename T, typename>
    // class icvPublisher;
    class icvMetaData;
    class icvFunction;

    // TODO: Add merge strategy to input ports
    ICV_ENUM_CLASS icvSubscriberMergeStrategy
    {
        Append,
        Intersect,
        Select,
    };

    class icvSubscriberInterface {
        public:
        virtual bool GetIsRepeatable()=0;
        virtual bool GetIsOptional()=0;
        virtual std::vector<icvPublisherInterface*>& GetConnections()=0;
        virtual Duration64 GetTimeTolerance()=0;
        virtual icvFunction* GetOwner()=0;
        virtual void SetIsRepeatable(const bool&)=0;
        virtual void SetIsOptional(const bool&)=0;
        virtual void SetTimeTolerance(const Duration64&)=0;

        virtual icvDataObject* RequireDataObject()=0;
        virtual void ReleaseDataObject()=0;
        virtual bool CheckUpdate()=0;
        virtual bool isconnected()=0;
        virtual void enable_Trigger()=0;
        virtual void disable_Trigger()=0;
        virtual bool is_Triggered()=0;
        virtual void setname(string)=0;
        virtual string getname()=0;
        virtual bool isSuitableDataType(icvDataObject*)=0;
        virtual bool isMatchingConnectionType()=0;
    };

    template <typename T, typename = typename std::enable_if<std::is_base_of<icv::core::icvDataObject, T>::value>::type>
    class icvSubscriber : public icvObject, public icvSubscriberInterface
    {
    public:
        icvSubscriber(icvFunction* owner, icv_shared_ptr<const icvMetaData> info)
        : _owner(owner)
        {
            // TODO: implement metadata assignment
        }
        icvSubscriber(icvFunction* owner)        
        : _owner(owner)
        {
            // TODO: implement metadata assignment
        }


        ICV_PROPERTY_GETSET(IsRepeatable, _repeatable, bool)
        ICV_PROPERTY_GETSET(IsOptional, _optional, bool)

        ICV_PROPERTY_GET(Connections, _connections, std::vector<icvPublisherInterface*>&)

        ICV_PROPERTY_GETSET(TimeTolerance, _tolerance, Duration64)
                //template <typename T>  T& update_Input() ;

        ICV_PROPERTY_GET(Owner, _owner, icvFunction*)

        bool isSuitableDataType(icvDataObject* dataToCheck)
        {
            if(dynamic_cast<T*>(dataToCheck)) return true;
            else return false;
        }
        bool isMatchingConnectionType() 
        {
            for(icvPublisherInterface* connection : _connections)
            {
                if(!CheckDataType<T>(connection,sizeof(T)))
                    return false;
            }
            return true;
        }
        icvDataObject* RequireDataObject()
        {

            if (_connections.size() == 0)
                return ICV_NULLPTR;
            if (_connections.size() == 1)
            {
                if(isMatchingConnectionType()){
                    switch (_connections.at(0)->GetBufferStrategy())
                    {
                    case icvPublisherBufferStrategy::Copied:

                    case icvPublisherBufferStrategy::Shm:

                    case icvPublisherBufferStrategy::BufferCopied:

                        if(!_connections[0]->GetSerializedBuffer().empty())
                        {
                            _serialized_buffer = _connections[0]->GetSerializedBuffer();
                            updateParsedDataObj();
                            return _dataCopy;
                        }
                        else return ICV_NULLPTR;
                        
                    case icvPublisherBufferStrategy::Direct:
                    case icvPublisherBufferStrategy::PointerCopy:
                        // return _connections[0]->ReadDataObject();

                    // TODO: Implement input queue
                    default:
                        break;
                    }
                } else {
                    ICV_LOG_WARN<<"icvFunction::icvSubscribe: the connected subscriber type doesn't match publisher's definition.";
                    return ICV_NULLPTR;
                }
            }
            //TODO: Process multiple connection
            return ICV_NULLPTR;
        }

        void ReleaseDataObject() // Release the lock of data object
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
        bool CheckUpdate() // Check and reset signal
        {
            bool update = _update;
            _update = false;
            return update;
        }
        bool isconnected() //   
        {
            if(GetConnections().size()>0)return true;
            else  return false;
        } 
        void enable_Trigger()
        {
           _Triggermode=true;
        }
        void disable_Trigger()
        {
           _Triggermode=false;
        }
        bool is_Triggered()
        {
            return _Triggermode;
        }
        void setname(string name)
        {
         to_pub_name=name;
        }
        string getname()
        {
         return to_pub_name;
        }


    private:
        icvFunction* _owner;
        std::vector<icvPublisherInterface*> _connections;
        bool _repeatable, _optional,_Triggermode=false;
        Duration64 _tolerance;
        bool _update;// Update signal
        bool _first_copy=true; 
        string to_pub_name;
        icvDataObject* _dataCopy = ICV_NULLPTR;
        std::string _serialized_buffer="";


        void updateParsedDataObj(){
            if(!_dataCopy) 
            {
                _dataCopy = new T;
            }
            std::stringstream ssBuf(_serialized_buffer);
            _dataCopy->Deserialize(ssBuf, 0);
        }

    //for temp test
    // private:
    //     friend class icvPublisher;
    };

    template<class T> bool CheckDataType(icvSubscriberInterface* inputPort)
    {
        for(icvPublisherInterface* connection : inputPort->GetConnections())
        { 
            if(CheckDataType<T>(connection,sizeof(T)))
                return false;
        }
        return true;
    }
}}

#endif // icvSubscriber_h
