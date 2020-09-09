#include "OpenICV/Extensions/PCL/icvPclVisualizerSink.h"
#include "OpenICV/Core/icvDataObject.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvPublisher.h"


#include <iostream>
namespace icv { 
    namespace pcl
{   


    ICV_REGISTER_FUNCTION(icvPclVisualizerSink)

    ICV_CONSTEXPR char KEY_IP_ADD[] = "ip_address";
    ICV_CONSTEXPR char KEY_WIDTH[] = "width";
    ICV_CONSTEXPR char KEY_HEIGHT[] = "height";

    icvPclVisualizerSink::icvPclVisualizerSink() : icvPclVisualizerSink(nullptr) {}
    icvPclVisualizerSink::icvPclVisualizerSink(icv_shared_ptr<const icvMetaData> info)
        : icvFunction( info)
    {
     if (_information.Contains(KEY_IP_ADD))
		ip_add.from_string(_information.GetString(KEY_IP_ADD));
		else ip_add.from_string("127.0.0.1");
		
		if (_information.Contains(KEY_WIDTH))
			width_ = _information.GetInteger(KEY_WIDTH);
		else width_ = 13600;

		if (_information.Contains(KEY_HEIGHT))
			height_ = _information.GetInteger(KEY_HEIGHT);
		else height_ = 1;
        
		Register_Sub("point_cloud");
        
    //Register_Sub("Random number");
    //Register_Sub_Remote("ipc://127.0.0.1:5555");
  
    }


    void icvPclVisualizerSink::Execute()
    {
        
        
        icv::pcl::icvPointCloudData<::pcl::PointXYZI> tempdata;
        ::pcl::PointCloud<::pcl::PointXYZI> cloud_receive;
		icvSubscribe("point_cloud",&tempdata);
        ::pcl::PointCloud<::pcl::PointXYZI>::Ptr  cloud_new (new ::pcl::PointCloud<::pcl::PointXYZI>);
        cloud_receive=tempdata.getvalue();
        //*cloud=temptoreceive;
        //icv::data::icvInt64Data a;
        //icvSubscribe<icv::data::icvInt64Data>("Random number");
        //icvSubscribe("Random number",&a);
        //icvSubscribe("point_cloud",&a);
        //int b=a.getvalue();
        // ICV_LOG_INFO<<"before : "<<b<<"  after : "<<3*b;

    }
}
}
