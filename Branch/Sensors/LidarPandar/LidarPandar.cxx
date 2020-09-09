//  @ Project : RADAR
//  @ File Name : sensorlrr.h
//  @ Date : 2016/12/22
//  @ Author :
//
//


#ifndef _LidarPandar_H
#define _LidarPandar_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
//#include "OpenICV/Net/icvUdpReceiverSource.h"
#include "OpenICV/Extensions/PCL/icvPointCloudData.hxx"
#include "OpenICV/Core/icvConfig.h"
#include "OpenICV/Core/icvTime.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>


#include <cstdlib>
#include <string>
#include <sstream>
#include <iostream>
#include <boost/thread/thread.hpp>
#include "PandarGrabber.h"
#include "rawdata.h"
#include <pthread.h>



//#include "boost_udp.h"

#define PI 3.1415926


typedef unsigned char uint8;
typedef float float32;
typedef double float64;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;



using namespace icv;
using namespace std;
using namespace core;
using namespace pandar_pointcloud;
static uint16_t PANDAR_POINT_SIZE = 36001; 
class LidarPandar: public icvFunction
{
public:
  LidarPandar(icv_shared_ptr<const icvMetaData> info) : icvFunction(0, 1, info)
  {

	if(_information.Contains("rpm"))config_1.rpm=_information.GetDecimal("rpm");
config_1.max_range=130;
config_1.min_range=0.9;
config_1.start_angle=0;
config_1.frame_id="pandar";
config_1.ip_port=8080;
config_1.read_fast=false;
config_1.repeat_delay=0.0;
config_1.read_once=false;
config_1.rpm=600;
    pthread_mutex_init(&datalock, NULL);
	pthread_mutex_init(&displaylock, NULL);
		cloud_viewer_ = boost::shared_ptr<::pcl::visualization::PCLVisualizer>(new ::pcl::visualization::PCLVisualizer("PCL Pandar Cloud"));

pandar_cloud_=boost::shared_ptr<pandar_rawdata::PPointCloud>(new pandar_rawdata::PPointCloud() );
pandar_cloud_out=boost::shared_ptr<pandar_rawdata::PPointCloud>(new pandar_rawdata::PPointCloud() );
		convert_=boost::shared_ptr<Convert>(new Convert(config_1)) ;
            ICV_LOG_WARN<<"OK"<<(int)pandar_cloud_->empty();


		viewpoint_ = true;
		if (viewpoint_)
			{
			

		//	handler_ = boost::shared_ptr<::pcl::visualization::PointCloudColorHandlerCustom<::pcl::PointXYZI>>(new ::pcl::visualization::PointCloudColorHandlerCustom<::pcl::PointXYZI>("intensity"));
			cloud_viewer_->addCoordinateSystem(3.0);
			cloud_viewer_->setBackgroundColor(0, 0, 0);
			cloud_viewer_->initCameraParameters();
			cloud_viewer_->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
			cloud_viewer_->setCameraClipDistances(0.0, 50.0);
			points_to_display=boost::shared_ptr< ::pcl::PointCloud<::pcl::PointXYZI>> (new ::pcl::PointCloud<::pcl::PointXYZI>());
			cloud_viewer_->addPointCloud<::pcl::PointXYZI> (points_to_display,  "Pandar");
			boost::thread showthread(boost::bind(&LidarPandar::showpointcloud, this,cloud_viewer_));

			}

	    }
 LidarPandar() : LidarPandar(nullptr) {}




 void copypointcloud(boost::shared_ptr< pandar_rawdata::PPointCloud> &cloud_in, boost::shared_ptr<::pcl::PointCloud<::pcl::PointXYZI>> &cloud_out)
 {

	//ICV_LOG_INFO<<"in width as : "<<cloud_in->points.size();

 cloud_out->points.resize(cloud_in->points.size()); 
 int temp=cloud_in->points.size();

//ICV_LOG_INFO<<"out size : "<<cloud_in->points.size();
  for (int i = 0; i < cloud_in->points.size(); i++) 
  { 
	
	cloud_out->points[i].x = cloud_in->points[i].x;
	cloud_out->points[i].y = cloud_in->points[i].y; 
	cloud_out->points[i].z = cloud_in->points[i].z; 
	cloud_out->points[i].intensity = cloud_in->points[i].intensity; 
 }
 cloud_out->width=cloud_in->points.size();
 cloud_out->height=1;
 cloud_out->is_dense=true;
//ICV_LOG_INFO<<"out width : "<<cloud_out->points.size();

}

 void showpointcloud(boost::shared_ptr<::pcl::visualization::PCLVisualizer> viewer)
 {
	 while(true)
	 {

	if (cloud_mutex_.try_lock ())


		{
			ICV_LOG_INFO<<"lock update ";

		viewer->spinOnce (100);

		cloud_mutex_.unlock();

		}

		boost::this_thread::sleep (boost::posix_time::microseconds (1000));

	}


 }


//   virtual void ConfigurateOutput(std::vector<icvPublisher*>& outputPorts) override {

// 		ini_Output(outputPorts,0);
// 			icvDataObject* outdataini=new icv::pcl::icvPointCloudData<::pcl::PointXYZI>(PANDAR_POINT_SIZE);
// outdataini->Reserve();
// 	  	outputPorts[0]->SetDataObject(outdataini );

//   }

//   virtual void ConfigurateInput(std::vector<icvSubscriber*>& inputPorts) override 
	  
//   {					ini_Input(inputPorts,0);

//   }

  virtual void Execute() override
    {


buff_Input(inputPorts);
buff_Output(outputPorts);


count_++;
	 
	if(count_==1)

	{
	}
//pthread_mutex_lock(&datalock);
//convert_->getpointcloud(pandar_cloud_);
pandar_cloud_=convert_->getpointcloud();
if(pandar_cloud_->points.size()>1)
{
	copypointcloud(pandar_cloud_,points_to_display);
			//ICV_LOG_INFO<<">>>   1 ";



}

//ICV_LOG_INFO<<"width   1 : "<<pandar_cloud_->points.size();
//pandar_cloud_->points;
//pandar_cloud_=convert_->outMsg;
//*pandar_cloud_out=*pandar_cloud_;
//cloud_mutex_.unlock ();
//pthread_mutex_unlock(&datalock);
//ICV_LOG_INFO<<"width   2 : "<<pandar_cloud_out->points.size();



    

	
			if (points_to_display->points.size()>1)//(pandar_cloud_dis)
	{


			
			
			if (viewpoint_)
			{


			boost::mutex::scoped_lock updatelock(cloud_mutex_);
			ICV_LOG_INFO<<"update ";
			

			cloud_viewer_->updatePointCloud<::pcl::PointXYZI>(points_to_display,"Pandar");
			updatelock.unlock();

			boost::this_thread::sleep (boost::posix_time::microseconds (100));
	

			}
	}

	
	


}

  

	
	
private:

  // boost::shared_ptr<PandarDriver> driver_;
  // boost::shared_ptr<InputSocket> socket_input;
  // boost::shared_ptr<InputPCAP> pcap_input;
  boost::shared_ptr<Convert> convert_;
  Config_pandar config_1;
  ::pcl::PointCloud<pandar_pointcloud::PointXYZIT> data_to_Send;
   boost::shared_ptr< ::pcl::PointCloud<::pcl::PointXYZI>> points_to_display;
  int count_=0;
  boost::mutex cloud_mutex_;
  bool viewpoint_=false ;
  static const int MaxUdpBufferSize = 1024;
  boost::shared_ptr<::pcl::visualization::PCLVisualizer> cloud_viewer_;
	boost::shared_ptr<::pcl::visualization::PointCloudColorHandler<::pcl::PointXYZI>> handler_;
  boost::shared_ptr<pandar_rawdata::PPointCloud> pandar_cloud_;
 boost::shared_ptr< pandar_rawdata::PPointCloud>   pandar_cloud_out;
      pthread_mutex_t datalock,displaylock;




};


ICV_REGISTER_FUNCTION(LidarPandar)

#endif 
