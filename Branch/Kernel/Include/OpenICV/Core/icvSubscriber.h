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
using namespace std;
namespace icv { namespace core
{
    class icvNode;
    class icvPublisher;
    class icvDataObject;
    class icvMetaData;
    class icvFunction;

    // TODO: Add merge strategy to input ports
    ICV_ENUM_CLASS icvSubscriberMergeStrategy
    {
        Append,
        Intersect,
        Select,
    };

    class icvSubscriber : public icvObject
    {
    public:
        icvSubscriber(icvFunction* owner, icv_shared_ptr<const icvMetaData> info);
        icvSubscriber(icvFunction* owner);


        ICV_PROPERTY_GETSET(IsRepeatable, _repeatable, bool)
        ICV_PROPERTY_GETSET(IsOptional, _optional, bool)

        ICV_PROPERTY_GET(Connections, _connections, std::vector<icvPublisher*>&)

        ICV_PROPERTY_GETSET(TimeTolerance, _tolerance, Duration64)
                //template <typename T>  T& update_Input() ;

        icvDataObject* RequireDataObject();
        void ReleaseDataObject(); // Release the lock of data object
        bool CheckUpdate(); // Check and reset signal
        bool isconnected(); //
        void enable_Trigger(); // 
        void disable_Trigger(); // 
        bool is_Triggered(); // 
        void setname(string name);
        string getname();



    private:
        icvFunction* _owner;
        std::vector<icvPublisher*> _connections;
        bool _repeatable, _optional,_Triggermode=false;
        Duration64 _tolerance;
        bool _update;// Update signal
        bool _first_copy=true; 
        string to_pub_name;
        icvDataObject* _dataCopy = ICV_NULLPTR;

    private:
        friend class icvPublisher;
    };

    template<class T> bool CheckDataType(icvSubscriber* inputPort)
    {
        for(icvPublisher* connection : inputPort->GetConnections())
           { if(CheckDataType<T>(connection,sizeof(T)))
                return true;
            else return false;
        }
    }
}}

#endif // icvSubscriber_h
