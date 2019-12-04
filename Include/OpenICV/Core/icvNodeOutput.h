#ifndef icvNodeOutput_h
#define icvNodeOutput_h

#define ICV_CONFIG_TYPEINDEX
#define ICV_CONFIG_THREAD
#include "OpenICV/Core/icvConfig.h"
#undef ICV_CONFIG_TYPEINDEX
#undef ICV_CONFIG_THREAD

#include "OpenICV/Core/icvObject.h"
#include "OpenICV/Core/icvMacros.h"
#include "OpenICV/Core/icvDataObject.h"
namespace icv { namespace core
{
   // class icvDataObject;
    class icvNode;
    class icvNodeInput;

    ICV_ENUM_CLASS icvNodeOutputFrequency
    {
        Lazy, // Sample frequency is larger than data change frequency
        Instant // Sample frequency is smaller than data change frequency
    };

    ICV_ENUM_CLASS icvNodeOutputBufferStrategy
    {
        Direct, // Output data buffer is locked when execution and then disposed
        Copied, // Output data buffer is copied before execution and then disposed
        /* Above strategies are forbiddened if the output have multiple connections */
        Buffered, // Output data buffer is locked when execution
        BufferCopied, // Output data buffer is copied before execution
        Queued, // Output data are queued and processed one by one
    };

    class icvNodeOutput : public icvObject
    {
    public:
        icvNodeOutput(icvNode* owner, icv_shared_ptr<const icvMetaData> info);
        icvNodeOutput(icvNode* owner);

        ICV_PROPERTY_GETSET(Frequency, _frequency, icvNodeOutputFrequency)
        ICV_PROPERTY_GETSET(BufferStrategy, _bufferStrategy, icvNodeOutputBufferStrategy)

        ICV_PROPERTY_GET(Connections, _connections, std::vector<icvNodeInput*>&)

        icvDataObject* RequireDataObject();
        icvDataObject* RequireDataObjectDeep(bool lock = true);

        void SetDataObject(icvDataObject* data);
        void ReleaseDataObject(); // Release the lock of data object
        void writedata();
        void Trigger();
        icvDataObject* WriteDataObject();
        icvDataObject* ReadDataObject();
        void flag_update();
        int get_flag();
        ICV_PROPERTY_GET(DataMutex, _dataMutex, icv_mutex&)

    private:
        icvNode* _owner;
        std::vector<icvNodeInput*> _connections;
        int bufferflag_=3;// the last finished writing flag
        icvNodeOutputFrequency _frequency;
        icvNodeOutputBufferStrategy _bufferStrategy = icvNodeOutputBufferStrategy::BufferCopied;
        icvDataObject* _data = ICV_NULLPTR;
        icvDataObject* buffer_1= ICV_NULLPTR, *buffer_2= ICV_NULLPTR, *buffer_3= ICV_NULLPTR, *buffer_input= ICV_NULLPTR;

        size_t datasize_;
        icv_mutex _dataMutex;
    };

    // TODO: support multiple type variants
    // TODO: support constructor params
    template<class T> bool CheckDataType(icvNodeOutput* output,size_t buffersize_)
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

#endif // icvNodeOutput_h
