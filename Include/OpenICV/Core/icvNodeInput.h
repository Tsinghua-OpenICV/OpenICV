#ifndef icvNodeInput_h
#define icvNodeInput_h

#include <vector>
#include "OpenICV/Core/icvObject.h"
#include "OpenICV/Core/icvMacros.h"

namespace icv { namespace core
{
    class icvNode;
    class icvNodeOutput;
    class icvDataObject;
    class icvMetaData;

    // TODO: Add merge strategy to input ports
    ICV_ENUM_CLASS icvNodeInputMergeStrategy
    {
        Append,
        Intersect,
        Select,
    };

    class icvNodeInput : public icvObject
    {
    public:
        icvNodeInput(icvNode* owner, icv_shared_ptr<const icvMetaData> info);
                icvNodeInput(icvNode* owner);


        ICV_PROPERTY_GETSET(IsRepeatable, _repeatable, bool)
        ICV_PROPERTY_GETSET(IsOptional, _optional, bool)

        ICV_PROPERTY_GET(Connections, _connections, std::vector<icvNodeOutput*>&)

        ICV_PROPERTY_GETSET(TimeTolerance, _tolerance, Duration64)

        icvDataObject* RequireDataObject();
        void ReleaseDataObject(); // Release the lock of data object
        bool CheckUpdate(); // Check and reset signal

    private:
        icvNode* _owner;
        std::vector<icvNodeOutput*> _connections;

        bool _repeatable, _optional;
        Duration64 _tolerance;
        bool _update;// Update signal
        bool _first_copy=true; 

        icvDataObject* _dataCopy = ICV_NULLPTR;

    private:
        friend class icvNodeOutput;
    };

    template<class T> bool CheckDataType(icvNodeInput* inputPort)
    {
        for(icvNodeOutput* connection : inputPort->GetConnections())
            if(!CheckDataType<T>(connection,sizeof(T)))
                return false;
        return true;
    }
}}

#endif // icvNodeInput_h
