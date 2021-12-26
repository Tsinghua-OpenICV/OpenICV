#ifndef icvReplay_h
#define icvReplay_h

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"
#include "OpenICV/Basis/icvMsgpackRecorder.h"
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
#include "OpenICV/Basis/icvMsgpackDeltaRecorder.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cstdio>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <algorithm>
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

ICV_CONSTEXPR char KEY_OUTPUT_FILEPATH[] = "path";
ICV_CONSTEXPR char KEY_PLAY_SPEED[] = "play speed";


namespace icv { namespace function
{
    using namespace std;
    using namespace icv::core;
    using namespace cv;
    typedef data::icvStructureData<Imu> icvImu;
    typedef data::icvStructureData<RadarLongOut>    icvlrr ;
    typedef data::icvStructureData<RadarSideOut>    icvsrr ;
    typedef data::icvStructureData<NavSatFix>    icvNavSatFix;
    typedef data::icvStructureData<TwistWithCovarianceStamped>    icvTwistWithCovarianceStamped;
    typedef data::icvStructureData<Odometry>    icvOdometry;
    class Replay : public icvFunction
    {
        public:
        Replay(icv_shared_ptr<const icvMetaData> info) : icvFunction(info) {
	    
     
        if (_information.Contains(KEY_OUTPUT_FILEPATH)){
            path = _information.GetString(KEY_OUTPUT_FILEPATH);
           }
        else ICV_LOG_FATAL<<"NO FILE PATH";
        if (_information.Contains(KEY_PLAY_SPEED))
        playspeed = std::min(_information.GetDecimal(KEY_PLAY_SPEED),1.00);
        else  playspeed=1;
        

        global_path=boost::filesystem::path(path);
        _recorder = new icvMsgpackDeltaRecorder(boost::filesystem::path(path), 0);
        cout<<"Recorder has recorded the number of topics: "<<_recorder->GetBufferKey().size()<<endl;
        
        topic_names=_recorder->GetBufferKey();
        for (int i=0;i<topic_names.size();i++){
            Register_Pub(topic_names[i]);
            cout<<"Topic names of the icvbag are: "<<topic_names[i]<<endl;
            }
        //_recorder = new _impl::icvMsgpackRecorder(boost::filesystem::system_complete(path), 0);
        Mintimeinfile_=_recorder->getheader().MinTime;
        Maxtimeinfile_=_recorder->getheader().MaxTime;
        cout<<"max time is "<<Maxtimeinfile_<<" and min time is "<<Mintimeinfile_<<endl;
        

        alldata_->emplace("Number",new data::icvInt64Data());
         alldata_->emplace("Random number",new data::icvInt64Data());
         alldata_->emplace("speed",new data::icvDoubleData());
         alldata_->emplace("SteeringAngle",new data::icvDoubleData());
        // alldata_->emplace("Number",new data::icvInt64Data());
        // alldata_->emplace("Random number",new data::icvInt64Data());

        // alldata_->emplace("robosense_points",new data::icvPointCloudData<pcl::PointXYZI>());
        //alldata_->emplace("GPS_data",new icvNavSatFix());
        // alldata_->emplace("IMU_data",new icvImu());
        // alldata_->emplace("first_cam",new data::icvCvMatData());
        // alldata_->emplace("odom_data",new icvOdometry());
        // alldata_->emplace("radar_front",new icvlrr());
        // alldata_->emplace("radar_rear",new icvlrr());
        // alldata_->emplace("radar_side1",new icvsrr());
        // alldata_->emplace("radar_side2",new icvsrr());
        // alldata_->emplace("radar_side3",new icvsrr());
        // alldata_->emplace("radar_side4",new icvsrr());
        // alldata_->emplace("vel_data",new icvTwistWithCovarianceStamped());

        lastrealtime_ = icvTime::time_us();
        lastreadtime_=Mintimeinfile_;
	}
    ~Replay(){
        ICV_LOG_INFO<<"End of Replay"<<endl;
    }
    void Execute() 
    {
        realtime_ = icvTime::time_us();
        time_t elapsedTime = realtime_ - lastrealtime_;
        
        time_t deltareadtime =  elapsedTime * playspeed;
        
        readtime_ = lastreadtime_ + deltareadtime;

	    //ICV_LOG_INFO << "readtime_:" << readtime_ << ", Maxtimeinfile:" << Maxtimeinfile_ << ", Mintimeinfile" << Mintimeinfile_ << ", end_:" << end_;
        if ((readtime_ <= Maxtimeinfile_) && (readtime_ >= Mintimeinfile_))
        {
            
           
            lastrealtime_ = realtime_ ;
            lastreadtime_ = readtime_ ; 
           
            for(auto topic_:topic_names)
            {
            
            if (alldata_->find(topic_)==alldata_->end()){
          
            ICV_LOG_WARN<<"The topic named "<<topic_<<" is not added to the alldata scope";
            }
            else{
                //std::cout<<"get source time: "<<alldata_->at(topic_)->GetSourceTime()<<endl;
            if(alldata_->at(topic_)->GetSourceTime() < readtime_) 
             { 
                    //cout<<"ckpt1: "<<topic_<<endl;
                    _recorder->PlayNext(alldata_->at(topic_), topic_);
                    //cout<<"ckpt2"<<endl;
                    // int b=alldata_->at(topic_)->As<data::icvInt64Data>().getvalue();
                    //ICV_LOG_INFO<<topic_<<" before : "<<b<<"  after : "<<3*b;
                    replay_count++;
                    ICV_LOG_INFO<<"playing: "<<topic_<<" number: "<<replay_count<<endl;
                    if(topic_=="first_cam")
                    { 
                            
                            // image_path_=global_path.parent_path()/topic_;
                            // string pictoread=alldata_->at(topic_)->As<data::icvCvMatData>().getImageName()+".jpg";
                            // image_filename=image_path_/pictoread;
                            //ICV_LOG_INFO<<"address: "<<image_filename.string();
                            read_img=alldata_->at(topic_)->As<data::icvCvMatData>().getvalue();
                            //read_img=cv::imread(image_filename.string());
                            count++;
                            imshow("replay image",read_img);
                            cv::waitKey(20);
                            //cout<<count<<endl;
                            //icvPublish(topic_,alldata_->at(topic_));
                

                    }
                    if(topic_=="pandar_points"||topic_=="robosense_points"){

                            point_cloud_path=global_path.parent_path()/topic_;
                            string pictoread=alldata_->at(topic_)->As<data::icvPointCloudData<pcl::PointXYZI>>().getPointCloudName()+".pcd";
                            image_filename=point_cloud_path/pictoread;
                            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
                            ICV_LOG_INFO<<"address: "<<image_filename.string();
                            pcl::io::loadPCDFile<pcl::PointXYZI> (image_filename.string(), *cloud);
                            pc_data=new data::icvPointCloudData<pcl::PointXYZI>();
                            pc_data->setvalue( *cloud);
                            pc_data->SetSourceTime(alldata_->at(topic_)->GetSourceTime());
                            icvPublish(topic_,pc_data);
                    }
                    else
                    {
                        icvPublish(topic_,alldata_->at(topic_));
                    }

                    //cout<<"number of pictures are: "<<output_count_<<endl;
             }      
             }
            }
        }   
            else {       
            if(end_)
            {
                lastrealtime_ = realtime_ ;
                ICV_LOG_INFO<<"End of file";
                end_=false;
                this->~Replay();
            }
            }         
  
    }
    
    

    private:
    int count=0;
    int replay_count=0;
    float playspeed;
    std::string path;
    bool end_=true;
    time_t lastreadtime_,lastrealtime_,readtime_,realtime_,Mintimeinfile_,Maxtimeinfile_;
    icv_map<std::string, icvDataObject*> * alldata_=new icv_map<string, icvDataObject*>();
    vector<string> topic_names;
    icvMsgpackDeltaRecorder* _recorder;
    data::icvCvMatData *img_data;
    boost::filesystem::path image_path_,image_filename,global_path,point_cloud_path;
    cv::Mat read_img;
    data::icvPointCloudData<pcl::PointXYZI> *pc_data;

    };
ICV_REGISTER_FUNCTION(Replay)

}
}  


#endif



