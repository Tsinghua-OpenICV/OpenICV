

#include "HesaiLidarSDK/include/hesaiLidarSDK.h"

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/structure/icvPointCloudData.hxx"
#include "OpenICV/Core/icvConfig.h"
#include "OpenICV/Core/icvTime.h"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <unistd.h>
using namespace icv;
using namespace std;
using namespace pandar_pointcloud;

inline unsigned long sec2us(double sec) {
    return (unsigned long)(sec * 1e6);
}
class LidarPandar: public icvFunction
{
public:
  LidarPandar(icv_shared_ptr<const icvMetaData> info) : icvFunction(info)
  {
    ICV_LOG_INFO<<"Lidar Pandar Driver Starting";
    Register_Pub("pandar_points");

      if(!pcapFile.empty())
    {
   
    
    
    hsdk = new HesaiLidarSDK(pcapFile, lidarCorrectionFile, (HesaiLidarRawDataSturct)laserReturnType, laserCount, (HesaiLidarPCLDataType)pclDataType,
                      boost::bind(&LidarPandar::lidarCallback, this, _1, _2));
    }
    else if(!serverIp.empty())
    {
      
      hsdk = new HesaiLidarSDK(lidarRecvPort, gpsPort, startAngle, lidarCorrectionFile,
                      boost::bind(&LidarPandar::lidarCallback, this, _1, _2),
                      NULL, (HesaiLidarRawDataSturct)laserReturnType, laserCount, (HesaiLidarPCLDataType)pclDataType);
    }

    hsdk->start();

    // pcapFile = "/home/pandora/Desktop/pandar40p.pcap";
    
   
  }
  virtual void Execute() override
    {
    //     ICV_LOG_INFO<<"Lidar Pandar Driver Executing";
   
    }
  

  void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp)
  {
     ICV_LOG_INFO<<"Receiving Pandar PointCloud";
    unsigned long srctime = icvTime::now_us().time_since_epoch().count();
    laserCloudIn->width=cld->width;
    laserCloudIn->is_dense=cld->is_dense;
    laserCloudIn->height=cld->height;
    laserCloudIn->points.resize(laserCloudIn->width,laserCloudIn->height);
    for (int i=0;i<cld->points.size();i++){
      //std::cout<<"point x: "<<cld->points[i].x<<" y: "<<cld->points[i].y<<" z: "<<cld->points[i].z<<endl;
     
      laserCloudIn->points[i].x=cld->points[i].x;
      laserCloudIn->points[i].y=cld->points[i].y;
      laserCloudIn->points[i].z=cld->points[i].z;
      laserCloudIn->points[i].intensity=cld->points[i].intensity;
     
    }
      std::cout<<"point size: "<<laserCloudIn->points.size()<<endl;
    // string filename="pandar_cloud/"+std::to_string(srctime)+".pcd";
    // ::pcl::io::savePCDFileASCII (filename,*laserCloudIn);
    icv::data::icvPointCloudData<::pcl::PointXYZI> pc;
    pc.setvalue(*laserCloudIn);
    pc.SetSourceTime(srctime);
    // std::cout<<"img source time: "<<srctime<<std::endl;
    icvPublish("pandar_points",&pc);

  }


private:
  //ros::Publisher lidarPublisher;
  //image_transport::Publisher imgPublishers[5];
  HesaiLidarSDK* hsdk;
  ::pcl::PointCloud<::pcl::PointXYZI>::Ptr laserCloudIn=::pcl::PointCloud<::pcl::PointXYZI>::Ptr(new ::pcl::PointCloud<::pcl::PointXYZI>);
  //laserCloudIn = ::pcl::PointCloud<::pcl::PointXYZI>::Ptr(new ::pcl::PointCloud<::pcl::PointXYZI>); 
  //boost::shared_ptr<::pcl::visualization::PCLVisualizer> cloud_viewer_ = boost::shared_ptr<::pcl::visualization::PCLVisualizer>(new ::pcl::visualization::PCLVisualizer("PCL Pandar Cloud"));
    string pcapFile="";
    string serverIp="192.168.1.201";
    int serverPort=9870;
    string calibrationFile="/config/calibration.yml";
    int lidarRecvPort=2368;
    int gpsPort=10110;
    int startAngle=0;
    string lidarCorrectionFile="config/correction.csv";
    int laserReturnType=1;
    int laserCount=40;
    int pclDataType=0;
};
ICV_REGISTER_FUNCTION(LidarPandar)