
#ifndef _HelloWorldFunction_H
#define _HelloWorldFunction_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"
#include "OpenICV/structure/icvPythonData.hxx"
//#include "OpenICV/Data/icvCvmatData.hxx"
#include "OpenICV/structure/CameraInfo.h"

#include <boost/thread/thread.hpp>
#include <iostream>
#include <mutex>
using namespace icv;
using namespace core;
using namespace std;

class HelloPython : public icvFunction
{
public:
    HelloPython(icv_shared_ptr<const icvMetaData> params) : icvFunction(params) {
	

		
    // Register_Sub("Random number");
    //Register_Pub_Remote("tcp://127.0.0.1:5555");
    //Register_Sub_Remote("tcp://127.0.0.1:5555");
    //Register_Pub_Remote("ipc:///tmp/buff_carla_ego_vehicle_camera_rgb_view_image_color");
    
	cam_info=new icv::data::CameraInfo();
    cam_info->header.seq=1;
    cam_info->width=1;
    cam_info->K[2]=1;
    tempdata=new icv::data::icvPythonData<icv::data::CameraInfo>();
    
    // Register_Pub ("first_cam");
    //Register_Pub_Remote("ipc://127.0.0.1:5555");
     Register_Pub_Remote("tcp://127.0.0.1:5555");
	}

    HelloPython() : HelloPython(nullptr) {	}

    virtual void Execute() override
    {
		    cam_info->header.seq+=1;
            cam_info->width+=1;
            cam_info->K[2]+=1;
            tempdata->setvaluePtr(cam_info);
           
			icvPublish_Remote("tcp://127.0.0.1:5555",tempdata); 
            cam_info=&tempdata->getvalue();
            sleep(1);
            //ICV_LOG_INFO<<"IN HELLO WORLD";
            ICV_LOG_INFO<<"seq:"<<cam_info->header.seq<<" "<<"width:"<<cam_info->width<<" K[2]:"<<cam_info->K[2];
            ICV_LOG_INFO<<"FrameID: "<<cam_info->header.frame_id;
			count_++ ;
           
		
    }
private:
int count_=0;
icv::data::icvPythonData<icv::data::CameraInfo> *tempdata; 
icv::data::CameraInfo *cam_info;
icv::data::PythonDataBase *base_info;

};

ICV_REGISTER_FUNCTION(HelloPython)


#endif  //
