#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
//#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/structure/icvPointCloudData.hxx"

//#include "icvPCLViewer.hxx"
//

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
//#include <pcl/io/vlp_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>

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
		if (_information.Contains("ip_address"))
		ip_add.from_string(_information.GetString("ip_address"));
		else
		 ip_add.from_string("192.168.111.203");
		
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
			boost::thread showthread(boost::bind(&LidarHDL::showpointcloud, this,cloud_viewer_));

			}


	}

void cloud_callback (const CloudConstPtr& cloud)
    {
      boost::mutex::scoped_lock lock (cloud_mutex_);
      cloud_ = cloud;
    }

//     virtual void ConfigurateInput(std::vector<icvSubscriber*>& inputPorts) override
//   {							ini_Input(inputPorts,0);
//     }
//      void ConfigurateOutput(std::vector<icvPublisher*>& outputPorts) 
//     {
// 										ini_Output(outputPorts,0);
// 			icvDataObject* outdataini=new icv::pcl::icvPointCloudData<::pcl::PointXYZI>(width_,height_);
// outdataini->Reserve();
// 		//ICV_LOG_TRACE << "Instantiated DataObjec at output";
// 	outputPorts[0]->SetDataObject(outdataini);
// 		//CheckDataType<icvDoubleData>(outputPorts[0]);
// 	}
 void showpointcloud(boost::shared_ptr<::pcl::visualization::PCLVisualizer> viewer)
 {
	 while(true)
	 {

	if (show_mutex.try_lock ())


		{
			ICV_LOG_INFO<<"lock update ";

		viewer->spinOnce (100);

		show_mutex.unlock();

		}

		boost::this_thread::sleep (boost::posix_time::microseconds (1000));

	}


 }
    virtual void Execute() override
    {


//buff_Input(inputPorts);
//buff_Output(outputPorts);
		// RequestSuspend(_interval);
		 //std::cout << "start"<< endl;

		count_++;

			// See if we can get a cloud
			if (cloud_mutex_.try_lock ())
			{
			cloud_.swap (cloud);
			cloud_mutex_.unlock ();
			}
	
			if (cloud)
			{
			//std::cout << "height  "<<cloud->height <<"width"<< cloud->width<<endl;
			///*(static_cast< icv::icvPointCloudData<pcl::PointXYZI>*>(outData[0])) = *cloud;

			//send_Output<icv::pcl::icvPointCloudData<::pcl::PointXYZI>>(0) = *cloud;
			tempdata.setoutvalue(*cloud);
            icvPublish("ploud_cloud",&tempdata);
			
			//_cap >> output;
			if (viewpoint_)
			{

			handler_->setInputCloud (cloud);
			boost::mutex::scoped_lock updatelock(show_mutex);

			if (!cloud_viewer_->updatePointCloud (cloud, *handler_, "HDL"))
			cloud_viewer_->addPointCloud (cloud, *handler_, "HDL");
			updatelock.unlock();

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
		

	  
			//return 0; // success
	}
	    private:
        int _interval = 100; 
			boost::shared_ptr<::pcl::visualization::PCLVisualizer> cloud_viewer_;
			boost::shared_ptr<::pcl::HDLGrabber> grabber_;
			boost::shared_ptr<::pcl::visualization::PointCloudColorHandler<::pcl::PointXYZI>> handler_;

			boost::mutex cloud_mutex_,show_mutex;
			std::string hdlCalibration, pcapFile; boost::asio::ip::address ip_add;
			bool viewpoint_; int width_, height_;
			boost::signals2::connection cloud_connection;
			CloudConstPtr cloud_, cloud;
			icv::pcl::icvPointCloudData<::pcl::PointXYZI> tempdata;
			//pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler_;
			//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler_;
			int count_ = 0;
};

ICV_REGISTER_FUNCTION(LidarHDL);

