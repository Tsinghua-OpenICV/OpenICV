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
namespace icv { namespace core
{
    using namespace std;
   // class icvDataObject;
    class icvNode;
    class icvSubscriber;

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

    class icvPublisher : public icvObject
    {
    public:
        icvPublisher(icvFunction* owner, icv_shared_ptr<const icvMetaData> info);
        icvPublisher(icvFunction* owner);

        ICV_PROPERTY_GETSET(Frequency, _frequency, icvPublisherFrequency)
        ICV_PROPERTY_GETSET(BufferStrategy, _bufferStrategy, icvPublisherBufferStrategy)

        ICV_PROPERTY_GET(Connections, _connections, std::vector<icvSubscriber*>&)

        icvDataObject* RequireDataObject();
        icvDataObject* RequireDataObjectDeep(bool lock = true);
        void set_send_data(icvDataObject* data_to_send);
        void Send_Out();

        void SetDataObject(icvDataObject* data);
        void ReleaseDataObject(); // Release the lock of data object
        void writedata();
        void Trigger();
        icvDataObject* WriteDataObject();
        icvDataObject* ReadDataObject();
        void flag_update();
        int get_flag();
        string getname(){return publishername_;};
        void setname(const string name_t){ publishername_=name_t;};

        ICV_PROPERTY_GET(DataMutex, _dataMutex, icv_mutex&)

    private:
        icvFunction* _owner;
        std::vector<icvSubscriber*> _connections;
        int bufferflag_=3;// the last finished writing flag
        icvPublisherFrequency _frequency;
        icvPublisherBufferStrategy _bufferStrategy = icvPublisherBufferStrategy::BufferCopied;
        icvDataObject* _data = ICV_NULLPTR;
        icvDataObject* buffer_1= ICV_NULLPTR, *buffer_2= ICV_NULLPTR, *buffer_3= ICV_NULLPTR, *buffer_input= ICV_NULLPTR;
        icvDataObject* data_to_send_=ICV_NULLPTR;
        icv_mutex dataoutlock;
        string publishername_;

        size_t datasize_;
        icv_mutex _dataMutex;
        spin_lock _lock;

        bool _first_send=true;
    };

    // TODO: support multiple type variants
    // TODO: support constructor params
    template<class T> bool CheckDataType(icvPublisher* output,size_t buffersize_)
    {
        if (output->RequireDataObject())
            return dynamic_cast<T*>(output->RequireDataObject());
        else
        {
            ICV_LOG_TRACE << "Instantiated" << icv_typeid(T).name() << " at output";
            output->SetDataObject((icvDataObject*)new T);
            return true;
        }
    }
}}

#endif // icvPublisher_h
