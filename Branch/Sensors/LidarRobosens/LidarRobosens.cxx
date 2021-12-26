//  @ Project : RADAR
//  @ File Name : sensorlrr.h
//  @ Date : 2016/12/22
//  @ Author :
//
//


#ifndef _LidarRSlidar_H
#define _LidarRSlidar_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
//#include "OpenICV/Net/icvUdpReceiverSource.h"
#include "OpenICV/structure/icvPointCloudData.hxx"
#include "OpenICV/Core/icvConfig.h"
#include "OpenICV/Core/icvTime.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>


#include <cstdlib>
#include <string>
#include <sstream>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pthread.h>
#include "rs_driver/api/lidar_driver.h"
#include <mutex>
#include <condition_variable>
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

using namespace robosense::lidar;

boost::shared_ptr<::pcl::PointCloud<::pcl::PointXYZI>> RSlidar_cloud_ = boost::shared_ptr<::pcl::PointCloud<::pcl::PointXYZI>>(new ::pcl::PointCloud<::pcl::PointXYZI>());
bool published_flag=true;
int msg_seq=0;
icv_mutex data_mutex;
icv_condition_variable data_var;
//static uint16_t RSlidar_POINT_SIZE = 36001; 
//std::unique_lock<std::mutex> ulockf(data_mutex);

bool is_publish_flag(){return published_flag;}
bool is_not_publish_flag(){return !published_flag;}




void pointCloudCallback(const PointCloudMsg<PointXYZI>& msg)
{
  /* Note: Please do not put time-consuming operations in the callback function! */
  /* Make a copy of the message and process it in another thread is recommended*/
    icv_unique_lock<icv_mutex> ulock(data_mutex);
    data_var.wait(ulock,is_publish_flag);
     

  RS_MSG << "msg: " << msg.seq << " point cloud size: " << msg.point_cloud_ptr->size() << RS_REND;
        unsigned long srctime = icvTime::now_us().time_since_epoch().count();
  msg_seq=msg.seq;
  
    RSlidar_cloud_->width=msg.width;
    RSlidar_cloud_->is_dense=msg.is_dense;
    RSlidar_cloud_->height=msg.height;
    RSlidar_cloud_->points.resize(msg.width*msg.height);


    for (int i=0;i<msg.point_cloud_ptr->size();i++){
      //std::cout<<"point x: "<<cld->points[i].x<<" y: "<<cld->points[i].y<<" z: "<<cld->points[i].z<<endl;
     
      RSlidar_cloud_->points[i].x=(*msg.point_cloud_ptr)[i].x;
      RSlidar_cloud_->points[i].y=(*msg.point_cloud_ptr)[i].y;
      RSlidar_cloud_->points[i].z=(*msg.point_cloud_ptr)[i].z;
      RSlidar_cloud_->points[i].intensity=(*msg.point_cloud_ptr)[i].intensity;
     
    }
    published_flag = false;
    data_var.notify_one();

    std::cout<<"point cloud size: "<<RSlidar_cloud_->points.size()<<endl;
  /*  string filename="point_cloud_robosense/"+std::to_string(srctime)+".pcd";
    ::pcl::io::savePCDFileASCII (filename,*RSlidar_cloud_);*/
   // publish_flag=false;
  
  
}

/**
 * @brief The exception callback function. This function will be registered to lidar driver.
 * @param code The error code struct.
 */
 void exceptionCallback(const Error& code)
{
  /* Note: Please do not put time-consuming operations in the callback function! */
  /* Make a copy of the error message and process it in another thread is recommended*/
  RS_WARNING << "Error code : " << code.toString() << RS_REND;
}


class LidarRSlidar: public icvFunction
{
public:
  LidarRSlidar(icv_shared_ptr<const icvMetaData> info) : icvFunction(info)
  {
  Register_Pub("robosense_points");
  RS_TITLE << "------------------------------------------------------" << RS_REND;
  RS_TITLE << "            RS_Driver Core Version: V " << RSLIDAR_VERSION_MAJOR << "." << RSLIDAR_VERSION_MINOR << "."
        << RSLIDAR_VERSION_PATCH << RS_REND;
  RS_TITLE << "------------------------------------------------------" << RS_REND;

  RSDriverParam param;                  ///< Create a parameter object
  param.input_param.msop_port = 6699;   ///< Set the lidar msop port number, the default is 6699
  param.input_param.difop_port = 7788;  ///< Set the lidar difop port number, the default is 7788
  param.lidar_type = LidarType::RS16;   ///< Set the lidar type. Make sure this type is correct
  param.print();
  
  //pcap read file mode
  // param.input_param.read_pcap = true;                              ///< Set read_pcap to true
  // param.input_param.pcap_path = "/home/robosense/rs16.pcap";  ///< Set the pcap file directory
  // param.input_param.device_ip = "192.168.1.200";  ///< Set the lidar ip address, the default is 192.168.1.200
  // param.input_param.msop_port = 6699;             ///< Set the lidar msop port number, the default is 6699
  // param.input_param.difop_port = 7788;            ///< Set the lidar difop port number, the default is 7788
  // param.lidar_type = LidarType::RS16;             ///< Set the lidar type. Make sure this type is correct
  // param.print();


  driver.regExceptionCallback(exceptionCallback);  ///< Register the exception callback function into the driver
  driver.regRecvCallback(pointCloudCallback);      ///< Register the point cloud callback function into the driver
  if (!driver.init(param))                         ///< Call the init function and pass the parameter
  {
    RS_ERROR << "Driver Initialize Error..." << RS_REND;
  }
  driver.start();  ///< The driver thread will start
  RS_DEBUG << "RoboSense Lidar-Driver Linux online driver start......" << RS_REND;
  driver.detach();
 //while (true){sleep(1);}

  //driver.stop();  ///< The driver thread will start


	}



void publishpointcloud(){
  //ICV_LOG_INFO<<"Publishing point cloud"<<endl;
   // if (!publish_flag){
      published_flag = true;
      unsigned long srctime = icvTime::now_us().time_since_epoch().count();

      icv::data::icvPointCloudData<::pcl::PointXYZI> pc;
      pc.setvalue( *RSlidar_cloud_);
      pc.SetSourceTime(srctime);
      // std::cout<<"img source time: "<<srctime<<std::endl;
      icvPublish("robosense_points",&pc);

      ICV_LOG_INFO<<"points size to be published "<<RSlidar_cloud_->points.size();
      published_flag = false;
      //  } 
}
    virtual void Execute() override
    {	 
        // icv_unique_lock<icv_mutex> ulock(data_mutex);
        // data_var.wait(ulock, []{return !published_flag;});
        icv_unique_lock<icv_mutex> ulock(data_mutex);
        data_var.wait(ulock,is_not_publish_flag);
        publishpointcloud();
        published_flag = true;
        data_var.notify_one();
         
      
    }

	
	




  

	
	
private:

  // boost::shared_ptr<RSlidarDriver> driver_;
  // boost::shared_ptr<InputSocket> socket_input;
  // boost::shared_ptr<InputPCAP> pcap_input;


//::pcl::PointCloud<RSlidar_pointcloud::PointXYZI> data_to_Send;
  LidarDriver<PointXYZI> driver;  ///< Declare the driver object

  bool viewpoint_=false ;
  static const int MaxUdpBufferSize = 1024;
  boost::shared_ptr<::pcl::visualization::PCLVisualizer> cloud_viewer_;
	boost::shared_ptr<::pcl::visualization::PointCloudColorHandler<::pcl::PointXYZI>> handler_;
 boost::shared_ptr< ::pcl::PointCloud<::pcl::PointXYZI>>   RSlidar_cloud_out;
 int last_seq=0;
  std::shared_ptr<std::thread> threads_publish;



};


ICV_REGISTER_FUNCTION(LidarRSlidar)

#endif 
