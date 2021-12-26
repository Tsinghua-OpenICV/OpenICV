#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
//#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/structure/icvPointCloudData.hxx"


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <pcl/io/pcd_io.h>

#include <boost/shared_ptr.hpp>
#include<deque>
//#include <pcl/io/vlp_grabber.h>

//#include <pcl/console/parse.h>


//using namespace pcl::console;
//using namespace pcl::visualization;
using namespace icv;
using namespace core;
class LidarHDL: public icvFunction
{
public:
		typedef ::pcl::PointCloud<::pcl::PointXYZI> Cloud;
		typedef typename Cloud::ConstPtr CloudConstPtr;
// constexpr const char KEY_IP_ADD[] = "ip_address";
// constexpr const char KEY_WIDTH[] = "width";
// constexpr const char KEY_HEIGHT[] = "height";

// constexpr const char KEY_INTERVAL[] = "interval";
LidarHDL(icv_shared_ptr <const icvMetaData> info) :  icvFunction( info){
	if (_information.Contains("interval"))
	{
		_interval = _information.GetInteger("interval");
		_information.Remove("interval");
	}
		if (_information.Contains("ip_address")){
		ip_add.from_string(_information.GetString("ip_address"));}
		else{
		 ip_add.from_string("192.168.111.203");}
		
		grabber_ = boost::shared_ptr<::pcl::HDLGrabber>(new ::pcl::HDLGrabber(ip_add, 2368, hdlCalibration));
		boost::function<void(const CloudConstPtr&)> cloud_cb = boost::bind(&LidarHDL::cloud_callback, this, _1);
		cloud_connection = grabber_->registerCallback(cloud_cb);
		grabber_->start();
		if (_information.Contains( "width"))
			width_ = _information.GetInteger( "width");
		else width_ = 13600;

		if (_information.Contains("height"))
			height_ = _information.GetInteger("height");
		else height_ = 1;
        Register_Pub("point_cloud");

		viewpoint_ = false;
		if (viewpoint_)
			{
			::pcl::visualization::PointCloudColorHandlerGenericField<::pcl::PointXYZI> color_handler("intensity");
			cloud_viewer_ = boost::shared_ptr<::pcl::visualization::PCLVisualizer>(new ::pcl::visualization::PCLVisualizer("PCL HDL Cloud"));
			handler_ = boost::shared_ptr<::pcl::visualization::PointCloudColorHandlerGenericField<::pcl::PointXYZI>>(new ::pcl::visualization::PointCloudColorHandlerGenericField<::pcl::PointXYZI>("intensity"));
			cloud_viewer_->addCoordinateSystem(3.0);
			cloud_viewer_->setBackgroundColor(0, 0, 0);
			cloud_viewer_->initCameraParameters();
			cloud_viewer_->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
			cloud_viewer_->setCameraClipDistances(0.0, 50.0);
			std::cout << "grabber pointer" << grabber_ << "   frames: " << grabber_->getFramesPerSecond() << endl;
			//boost::thread showthread(boost::bind(&LidarHDL::showpointcloud, this,cloud_viewer_));

			}


	}

void cloud_callback (const CloudConstPtr& cloud)
    {
	ICV_LOG_INFO<<"Starting to cloud callback";
    in_cloud = cloud;
	for (int i=0;i<=in_cloud->points.size();i++){
		
	}
	  ICV_LOG_INFO<<"Ending to cloud callback";
    }


 void showpointcloud(boost::shared_ptr<::pcl::visualization::PCLVisualizer> viewer)
 {
	 while(true)
	 {
    ICV_LOG_INFO<<"Starting to showpointcloud";
		//viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (1000));
          ICV_LOG_INFO<<"Ending to showpointcloud";
	}


 }
    virtual void Execute() override
    {

    ICV_LOG_INFO<<"Starting to execute";
		count_++;

			// See if we can get a cloud
		
			if (in_cloud)
			{
			unsigned long srctime = icvTime::now_us().time_since_epoch().count();

			std::cout << "height  "<<in_cloud->height <<"width"<< in_cloud->width<<endl;
			///*(static_cast< icv::icvPointCloudData<pcl::PointXYZI>*>(outData[0])) = *cloud;
            string filename="pc_hdl/"+std::to_string(srctime)+".pcd";
            ::pcl::io::savePCDFileASCII (filename,*in_cloud);
			//send_Output<icv::data::icvPointCloudData<::pcl::PointXYZI>>(0) = *cloud;
			pc=*in_cloud;
			pc.SetSourceTime(srctime);

            icvPublish("ploud_cloud",&pc);
			
			//_cap >> output;
			if (viewpoint_)
			{

			handler_->setInputCloud (in_cloud);
			if (!cloud_viewer_->updatePointCloud (in_cloud, *handler_, std::to_string(srctime))){
			cloud_viewer_->spinOnce ();
			cloud_viewer_->removePointCloud(std::to_string(srctime));  

			}
			}
			}


			
			if (viewpoint_)
			{
			if (cloud_viewer_->wasStopped())
			{
			grabber_->stop();

			cloud_connection.disconnect();
			}
			}
		
  //      ICV_LOG_INFO<<"Ending to execute";

	  
			//return 0; // success
	}
	    private:
            int _interval = 100; 
			boost::shared_ptr<::pcl::visualization::PCLVisualizer> cloud_viewer_;
			boost::shared_ptr<::pcl::HDLGrabber> grabber_;
			boost::shared_ptr<::pcl::visualization::PointCloudColorHandler<::pcl::PointXYZI>> handler_;

			std::string hdlCalibration, pcapFile; boost::asio::ip::address ip_add;
			bool viewpoint_; int width_, height_;
			boost::signals2::connection cloud_connection;
			pcl::PointCloud<::pcl::PointXYZI>::ConstPtr in_cloud;
			icv::data::icvPointCloudData<::pcl::PointXYZI> pc;
			//pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler_;
			//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler_;
			int count_ = 0;
};

ICV_REGISTER_FUNCTION(LidarHDL);


