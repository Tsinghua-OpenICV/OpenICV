
#include "OpenICV/Extensions/PCL/icvHDLGrabberSource.h"
#include "OpenICV/structure/icvPointCloudData.hxx"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Core/icvSubscriber.h"
#include <iostream>
using namespace std;


namespace icv
{
    namespace pcl
    {
        
        ICV_REGISTER_FUNCTION(icvHDLGrabberSource)

        ICV_CONSTEXPR char KEY_IP_ADD[] = "ip_address";
        ICV_CONSTEXPR char KEY_WIDTH[] = "width";
        ICV_CONSTEXPR char KEY_HEIGHT[] = "height";
        //boost::shared_ptr<const icv::core::icvMetaData> or icv_shared_ptr<const icvMetaData>?
        icvHDLGrabberSource::icvHDLGrabberSource() : icvHDLGrabberSource(ICV_NULLPTR) {}
        icvHDLGrabberSource::icvHDLGrabberSource(icv_shared_ptr<const icvMetaData> info)
            : icvFunction(info)
        {
        if (_information.Contains(KEY_IP_ADD))
		ip_add.from_string(_information.GetString(KEY_IP_ADD));
		else
		 ip_add.from_string("127.0.0.1");
		
		grabber_ = boost::shared_ptr<::pcl::HDLGrabber>(new ::pcl::HDLGrabber(ip_add, 2368, hdlCalibration));
		boost::function<void(const CloudConstPtr&)> cloud_cb = boost::bind(&icvHDLGrabberSource::cloud_callback, this, _1);
		cloud_connection = grabber_->registerCallback(cloud_cb);
		grabber_->start();
		if (_information.Contains(KEY_WIDTH))
			width_ = _information.GetInteger(KEY_WIDTH);
		else width_ = 13600;

		if (_information.Contains(KEY_HEIGHT))
			height_ = _information.GetInteger(KEY_HEIGHT);
		else height_ = 1;
        Register_Pub("point_cloud");
        }
        void icvHDLGrabberSource::cloud_callback (const CloudConstPtr& cloud)
        {
        boost::mutex::scoped_lock lock (cloud_mutex_);
        cloud_ = cloud;
        }
     

        void icvHDLGrabberSource::Execute()
    {
		::pcl::PointCloud<::pcl::PointXYZI>::Ptr  cloud_new (new ::pcl::PointCloud<::pcl::PointXYZI>);
		//cloud_new(new ::pcl::PointCloud<::pcl::PointXYZI>);
        if (::pcl::io::loadPCDFile<::pcl::PointXYZI>("/home/yining/Desktop/pcd/1.pcd", *cloud_new) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
	}
	    else
	{
		std::cout << "Cloud reading succeeded." << std::endl;
	}

	
        count_++;
		if(count_==1)
		{


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
			}
		}


			if(count_>=1)
			{
       
	
	 
			// See if we can get a cloud
			if (cloud_mutex_.try_lock ())
			{
			cloud_.swap (cloud);
			cloud_mutex_.unlock ();
			}
	
			if (cloud_new)
			{
			//std::cout << "height  "<<cloud->height <<"width"<< cloud->width<<endl;
			//(static_cast< icv::icvPointCloudData<pcl::PointXYZI>*>(outData[0])) = *cloud;
			//@deprecated
			
			//send_Output<icv::pcl::icvPointCloudData<::pcl::PointXYZI>>(0) = *cloud;
			//Send_Out(tempdata,0);
            //icv::pcl::icvPointCloudData<int>&
            //ICV_LOG_INFO("sent out cloud");
			cout<<"sent out cloud"<<endl;
            tempdata.setoutvalue(*cloud_new);
            icvPublish("ploud_cloud",&tempdata);
			//_cap >> output;
			if (viewpoint_)
			{
			handler_->setInputCloud (cloud);
			if (!cloud_viewer_->updatePointCloud (cloud_new, *handler_, "HDL"))
			cloud_viewer_->addPointCloud (cloud_new, *handler_, "HDL");

			cloud_viewer_->spinOnce ();
			}
			}

			if (!grabber_->isRunning())
			{
			if (viewpoint_)cloud_viewer_->spin();
			}

			boost::this_thread::sleep (boost::posix_time::microseconds (100));
			}
			if (viewpoint_)
			{
			if (cloud_viewer_->wasStopped())
			{
			grabber_->stop();

			cloud_connection.disconnect();
			}
			}
        }
    }
}


