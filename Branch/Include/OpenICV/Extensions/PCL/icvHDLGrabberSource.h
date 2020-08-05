#ifndef icvHDLGrabberSource_h
#define icvHDLGrabberSource_h

#include "OpenICV/Core/icvFunction.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
//#include <pcl/io/vlp_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include "OpenICV/Extensions/PCL/icvPointCloudData.hxx"

namespace icv
{
    namespace pcl
    {
                using namespace icv::core;

        class icvHDLGrabberSource : public icvFunction
        {
        public:
        typedef ::pcl::PointCloud<::pcl::PointXYZI> Cloud;
		typedef typename Cloud::ConstPtr CloudConstPtr;
            icvHDLGrabberSource();
            icvHDLGrabberSource(icv_shared_ptr<const icvMetaData> info);
            
       

            virtual void Execute() ICV_OVERRIDE;
		    void cloud_callback(const CloudConstPtr& cloud);

        private:
            std::string correction_file, _ipaddr;
            unsigned short port;
            int _interval = 100; 
			boost::shared_ptr<::pcl::visualization::PCLVisualizer> cloud_viewer_;
			boost::shared_ptr<::pcl::HDLGrabber> grabber_;
			boost::shared_ptr<::pcl::visualization::PointCloudColorHandler<::pcl::PointXYZI>> handler_;

			boost::mutex cloud_mutex_;
			std::string hdlCalibration, pcapFile; boost::asio::ip::address ip_add;
			bool viewpoint_; int width_, height_;
			boost::signals2::connection cloud_connection;
			CloudConstPtr cloud_, cloud;
            icv::pcl::icvPointCloudData<::pcl::PointXYZI> *tempdata;
			//pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler_;
			//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler_;
			int count_ = 0;
        };
    }
}

#endif // icvHDLGrabberSource_h
