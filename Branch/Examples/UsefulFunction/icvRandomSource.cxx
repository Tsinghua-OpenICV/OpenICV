#include "icvRandomSource.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"

#include <random>

using namespace icv::data;

namespace icv { namespace function
{
    ICV_REGISTER_FUNCTION(icvRandomSource)

    ICV_CONSTEXPR char KEY_INTERVAL[] = "interval";

    icvRandomSource::icvRandomSource() : icvRandomSource(ICV_NULLPTR) {}
    icvRandomSource::icvRandomSource(icv_shared_ptr<const icvMetaData> info)
        : icvFunction(info)
    {
        if (_information.Contains(KEY_INTERVAL))
        {
            _interval = _information.GetInteger(KEY_INTERVAL);
            _information.Remove(KEY_INTERVAL);
        }
        //int parameter=_information.GetInteger("para");
        Register_Pub ("Random number");
        Register_Pub ("SteeringAngle");
        Register_Pub ("speed");

        //Register_Pub ("Random number2");
        //Register_Pub ("Random number3");
        //Register_Pub_Remote("ipc://127.0.0.1:5555");
        Register_Pub_Remote("tcp://127.0.0.1:2000");
//run once when initialized
        
    }

    void icvRandomSource::Execute()
    {

// run cycle
        double angle=rand() % 256;
        double speed=rand() % 256;

        int64_t data = rand() % 256;

       // tempdata->setoutvalue(data);
       // long t_res=get_loop_time(false);
       // time_t temp1=icvTime::time_ns();
       //datasend.setvalue(data);
        data::icvInt64Data datatosend(data);
        data::icvDoubleData datatosend_angle(angle);
        data::icvDoubleData datatosend_speed(speed);
       
        //icvPublish_Remote("ipc://127.0.0.1:5555",&datatosend);

        //ICV_LOG_INFO<<"send data: "<<data;
        icvPublish("Random number",&datatosend);//=data;
        icvPublish("SteeringAngle",&datatosend_angle);//=data;
        icvPublish("speed",&datatosend_speed);//=data;

        //icvPublish("Random number2",&datatosend);//=data;
        //icvPublish("Random number3",&datatosend);//=data;
        icvPublish_Remote("tcp://127.0.0.1:2000",&datatosend);

    }
}}
