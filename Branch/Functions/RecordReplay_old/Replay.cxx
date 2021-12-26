#ifndef icvReplay_h
#define icvReplay_h

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"
#include "OpenICV/Basis/icvMsgpackRecorder_old.h"
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <boost/filesystem/convenience.hpp>
#include "OpenICV/Basis/icvSemaphore.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/structure/icvCvMatData.hxx"
#include "OpenICV/Basis/icvStructureData.hxx"
#include "OpenICV/structure/structureCanTuan.h"
#include "OpenICV/structure/structureVehinfo.h"
#include "OpenICV/structure/structure_gps.h"
#include "OpenICV/structure/structureIMU.h"
#include "OpenICV/structure/RadarRecordReplay.h"
#include "OpenICV/structure/icvPointCloudData.hxx"
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>

ICV_CONSTEXPR char KEY_OUTPUT_FILEPATH[] = "path";

    using namespace std;
    using namespace icv;
    typedef data::icvStructureData<Imu> icvImu;
    typedef data::icvStructureData<RadarLongOut>    icvlrr ;
    typedef data::icvStructureData<RadarSideOut>    icvsrr ;
    typedef data::icvStructureData<NavSatFix>    icvNavSatFix;
    typedef data::icvStructureData<TwistWithCovarianceStamped>    icvTwistWithCovarianceStamped;
    typedef data::icvStructureData<Odometry>    icvOdometry;
    class Replay_old : public icvFunction
    {
        public:
        Replay_old(icv_shared_ptr<const icvMetaData> info) : icvFunction(info) {
	
        std::string path;
        if (_information.Contains(KEY_OUTPUT_FILEPATH))
            path = _information.GetString(KEY_OUTPUT_FILEPATH);
            
        else ICV_LOG_FATAL<<"NO FILE PATH";
        image_path_=boost::filesystem::path(path);
        image_path_=image_path_.parent_path()/"image";
        pc_path=boost::filesystem::path(path);
        pc_path=pc_path.parent_path()/"pc_hdl";

        _recorder = new icvMsgpackRecorder_old(boost::filesystem::system_complete(path), 0);
        Mintimeinfile_=_recorder->getheader().MinTime;
        Maxtimeinfile_=_recorder->getheader().MaxTime;
        //cout<<"max time is "<<Maxtimeinfile_<<" and min time is "<<Mintimeinfile_<<endl;
        icv_thread loop(bind(&Replay_old::InnerLoop, this));
        timesync_=new icvSemaphore(1);
      
        current_data=new data::icvCvMatData ();
        last_data=new data::icvCvMatData();
        img_data = new data::icvCvMatData ();
        last_data1=new icvImu();
        last_data2=new icvNavSatFix();
        last_data3=new icvTwistWithCovarianceStamped();
        last_data4=new icvOdometry();
        last_data5=new icvlrr();
        last_data6=new icvlrr();
        last_data7=new icvsrr();
        last_data8=new icvsrr();
        last_data9=new icvsrr();
        last_data10=new icvsrr();
        last_data11=new data::icvPointCloudData<pcl::PointXYZI>();
        
        lastrealtime_ = icvTime::now_us().time_since_epoch().count();
        lastreadtime_=Mintimeinfile_;
        //boost::shared_ptr<::pcl::visualization::PCLVisualizer> cloud_viewer_ = boost::shared_ptr<::pcl::visualization::PCLVisualizer>(new ::pcl::visualization::PCLVisualizer("PCL velodyne Cloud"));

        ICV_LOG_INFO<<"time min"<<Mintimeinfile_;
        
        last_data->SetSourceTime(Mintimeinfile_);
        // cv::namedWindow("repaly1");		
        Register_Pub("first_cam");
        Register_Pub("IMU_data");
        Register_Pub("GPS_data");
        Register_Pub("vel_data");
        Register_Pub("odom_data");
        Register_Pub("radar_front");
        Register_Pub("radar_rear");
        Register_Pub("radar_side1");
        Register_Pub("radar_side2");
        Register_Pub("radar_side3");
        Register_Pub("radar_side4");
        //Register_Pub("velodyne_points");

	}


    void Execute() 
    {
        count_++ ;
        timesync_->Lock();
      
        realtime_ = icvTime::time_us();
        time_t elapsedTime = realtime_ - lastrealtime_;
        float playspeed=1;
        time_t deltareadtime =  elapsedTime * playspeed;

        readtime_ = lastreadtime_ + deltareadtime;
        samplesofdelatT[count_% 1000] = elapsedTime;
      
	    ICV_LOG_INFO << "readtime_:" << readtime_ << ", Maxtimeinfile:" << Maxtimeinfile_ << ", Mintimeinfile" << Mintimeinfile_ << ", end_:" << end_;

        if ((readtime_ <= Maxtimeinfile_) && (readtime_ >= Mintimeinfile_)) {
          
            lastrealtime_ = realtime_ ;
            lastreadtime_ = readtime_ ; 
            /* play camera data */ 
            if(last_data->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data, 0);
                string pictoread=last_data->getImageName()+".jpg";
                image_filename=image_path_/pictoread;
                // ICV_LOG_INFO<<"address"<<image_filename.string();
                read_img=cv::imread(image_filename.string());
                //imshow("Replaying Image",read_img);
		        //cv::waitKey(20);
                img_data->setvalue(read_img);
                img_data->SetSourceTime(last_data->GetSourceTime());
                //cout<<"The source time sent is "<<img_data->GetSourceTime()<<endl;
                icvPublish("first_cam",img_data);      
                ICV_LOG_INFO<<"lastdata1.sourcetime: "<<last_data->GetSourceTime()<<endl;

            }
            
            /* play IMU data */

            if(last_data1->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data1, 1);
                icvPublish("IMU_data",last_data1);
                ICV_LOG_INFO<<"lastdata2.sourcetime: "<<last_data1->GetSourceTime()<<endl;
                //ICV_LOG_INFO<<"replay IMU";

            }
            /* play GPS data */
            if(last_data2->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data2, 2);
                icvPublish("GPS_data",last_data2);
                ICV_LOG_INFO<<"lastdata3.sourcetime: "<<last_data2->GetSourceTime()<<endl;
               // ICV_LOG_INFO<<"replay gps";
            }
            
            /* play Vel data */
            if(last_data3->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data3, 3);
                 icvPublish("vel_data",last_data3);
                ICV_LOG_INFO<<"lastdata4.sourcetime: "<<last_data3->GetSourceTime()<<endl;
               // ICV_LOG_INFO<<"replay vel";
            }
            
            /* play Odom data */
            if(last_data4->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data4, 4);
                 icvPublish("odom_data",last_data4);
               // ICV_LOG_INFO<<"replay odom";
            }
            /* play LRR1 front data */
            if(last_data5->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data5, 5);
               
                icvPublish("radar_front",last_data5);
               // ICV_LOG_INFO<<"replay radar front";
            }
            /* play LRR2 rear data */
            if(last_data6->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data6, 6);
                icvPublish("radar_rear",last_data6);
               // ICV_LOG_INFO<<"replay radar rear";
            }
            /* play SRR1 front left data */
            if(last_data7->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data7, 7);
                icvPublish("radar_side1",last_data7);
              //  ICV_LOG_INFO<<"replay radar side 1";
            }
            /* play SRR2 front right data */
            if(last_data8->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data8, 8);
                icvPublish("radar_side2",last_data8);
              //  ICV_LOG_INFO<<"replay radar side 2";
            }
            /* play SRR3 rear left data */
            if(last_data9->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data9, 9);
                icvPublish("radar_side3",last_data9);
               // ICV_LOG_INFO<<"replay radar side 3";
            }
            /* play SRR4 rear right data */
            if(last_data10->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data10, 10);
                icvPublish("radar_side4",last_data10);
               // ICV_LOG_INFO<<"replay radar side 4";
            }
             ICV_LOG_INFO<<"lastdata11.sourcetime: "<<last_data10->GetSourceTime()<<endl;

            // if(last_data11->GetSourceTime() < readtime_) 
            // {
            //     ICV_LOG_INFO<<"Starting to replay lidar message";
            //     // cloud_viewer_->addCoordinateSystem(3.0);
            //     // cloud_viewer_->setBackgroundColor(0, 0, 0);
            //     // cloud_viewer_->initCameraParameters();
            //     // cloud_viewer_->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
            //     // cloud_viewer_->setCameraClipDistances(0.0, 50.0);      
            //     _recorder->PlayNext(last_data11, 11);
            //     string pcd_name=last_data11->GetSourceTime()+".pcd";
            //     pc_filename=pc_path/pcd_name;
            //     ::pcl::io::loadPCDFile<::pcl::PointXYZI>(pc_filename.string(), *read_pcd);
            //     // cloud_viewer_->addPointCloud<::pcl::PointXYZI> (read_pcd,std::to_string(last_data->GetSourceTime()));
            //     // cloud_viewer_->spinOnce ();
            //     // cloud_viewer_->removePointCloud(std::to_string(last_data->GetSourceTime()));  
            //     // ICV_LOG_INFO<<"address"<<image_filename.string();
            //     icv::data::icvPointCloudData<::pcl::PointXYZI> pc;
            //     pc = *read_pcd;
            //     pc.SetSourceTime(last_data->GetSourceTime());
            //     // std::cout<<"img source time: "<<srctime<<std::endl;
            //     icvPublish("velodyne_points",&pc);
            //    // ICV_LOG_INFO<<"replay radar side 4";
            // }
        }
        else {       
            if(end_)
            {
                lastrealtime_ = realtime_ ;
                ICV_LOG_INFO<<"End of file";
                end_=false;
            }
        }
    }
    private:
    void InnerLoop()
    {
        while(true) 
        {
            timesync_->Release();
            usleep(ICV_SLEEP_CYCLE);
        }
    }

    private:
    bool first_=true,end_=true;
    int output_count_;
    time_t lastreadtime_,lastrealtime_,readtime_,realtime_,Mintimeinfile_,Maxtimeinfile_;
    boost::shared_ptr<::pcl::visualization::PCLVisualizer> cloud_viewer_;
    vector<string> typenames_;
    int count_=0;
    float samplesofdelatT[1000];
    icvMsgpackRecorder_old* _recorder;
    icvSemaphore* timesync_;
    data::icvCvMatData* current_data,*last_data, *img_data;
    icvImu* last_data1 ;
    icvNavSatFix* last_data2 ;
    icvTwistWithCovarianceStamped* last_data3;
    icvOdometry* last_data4;
    icvlrr* last_data5 ;
    icvlrr* last_data6 ;
    icvsrr* last_data7 ;
    icvsrr* last_data8 ;
    icvsrr* last_data9 ;
    icvsrr* last_data10 ;
    ::pcl::PointCloud<::pcl::PointXYZI>::Ptr read_pcd=::pcl::PointCloud<::pcl::PointXYZI>::Ptr(new ::pcl::PointCloud<::pcl::PointXYZI>);

    data::icvPointCloudData<pcl::PointXYZI> *last_data11;
    data::icvInt64Data* tempdata1;
    boost::filesystem::path image_path_,image_filename,pc_path,pc_filename;
    cv::Mat read_img;
    };
ICV_REGISTER_FUNCTION(Replay_old)


#endif



