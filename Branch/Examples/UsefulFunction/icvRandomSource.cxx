#include "icvRandomSource.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Basis/icvProtoBufData.hxx"
#include "OpenICV/Basis/icvProtoBufData.pb/icvTestData.pb.h"
#include <random>
#include <unistd.h>
using namespace icv::data;

namespace icv { namespace function
{
    typedef icvPBDataPkg::icvTest2PbMsg test_type;
    typedef icv::data::icvProtoBufData<test_type> AS;

    ICV_REGISTER_FUNCTION(icvRandomSource)

    ICV_CONSTEXPR char KEY_INTERVAL[] = "interval";

    icvRandomSource::icvRandomSource() : icvRandomSource(ICV_NULLPTR) {}
    icvRandomSource::icvRandomSource(icv_shared_ptr<const icvMetaData> info)
        : icvFunction(info)
    {
        std::cout<<"print something";
        if (_information.Contains(KEY_INTERVAL))
        {
            _interval = _information.GetInteger(KEY_INTERVAL);
            _information.Remove(KEY_INTERVAL);
        }
        //int parameter=_information.GetInteger("para");

        Register_Pub<AS>("Random number");
        Register_Pub<icv::data::icvDoubleData>("speed");

        // Register_Pub ("SteeringAngle");
        // Register_Pub ("speed");

        //Register_Pub ("Random number2");
        //Register_Pub ("Random number3");
        //Register_Pub_Remote("ipc://127.0.0.1:5555");
        // Register_Pub_Remote("tcp://127.0.0.1:2000");
//run once when initialized
        
    }

    void icvRandomSource::Execute()
    {
        // std::cout << typeid(std::enable_if<std::is_base_of<icv::core::icvDataObject, AS>::value, bool>::type).name() << std::endl;


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

        
        test_type a_data;
        a_data.set_id(rand() % 256);
        // a_data.set_name("Jerry");
        AS test_type(a_data);
        std::cout  <<"Publisher[Random number]:  "<< test_type.getvalue().id() <<std::endl;
        // std::cout  <<"new Publisher:  "<< data <<std::endl;

       
        //icvPublish_Remote("ipc://127.0.0.1:5555",&datatosend);

        //ICV_LOG_INFO<<"send data: "<<data;
        //icvPublish("Random number",&test_type);//=data;
        // icvPublish("Random number", &datatosend);//=data;
        // icvPublish("SteeringAngle",&datatosend_angle);//=data;
        sleep(1);
        icvPublish("speed",&datatosend_speed);//=data;
        std::cout <<"Publisher[speed]:  "<< datatosend_speed.getvalue() <<std::endl;

        //icvPublish("Random number2",&datatosend);//=data;
        //icvPublish("Random number3",&datatosend);//=data;
        // icvPublish_Remote("tcp://127.0.0.1:2000",&datatosend);
        sleep(1);
    }
}}
