#ifndef icvReplay_h
#define icvReplay_h

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
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

ICV_CONSTEXPR char KEY_OUTPUT_FILEPATH[] = "path";


namespace icv { namespace function
{
    using namespace std;
    using namespace icv::core;
    using namespace _impl;
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
	
        std::string path;
        if (_information.Contains(KEY_OUTPUT_FILEPATH))
            path = _information.GetString(KEY_OUTPUT_FILEPATH);
        else ICV_LOG_FATAL<<"NO FILE PATH";
        image_path_=boost::filesystem::path(path);
        image_path_=image_path_.parent_path()/"image";

        _recorder = new _impl::icvMsgpackRecorder(boost::filesystem::system_complete(path), 0);
        Mintimeinfile_=_recorder->getheader().MinTime;
        Maxtimeinfile_=_recorder->getheader().MaxTime;

        icv_thread loop(bind(&Replay::InnerLoop, this));
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
        last_data0=new icvsrr();
        
        lastrealtime_ = SyncClock::now_us().time_since_epoch().count();
        lastreadtime_=Mintimeinfile_;

        // ICV_LOG_INFO<<"time min"<<Mintimeinfile_;
        
        last_data->SetSourceTime(Mintimeinfile_);
        // cv::namedWindow("repaly1");		
	}


    void Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts) 
    {
        count_++ ;
        timesync_->Lock();
      
        realtime_ = SyncClock::now_us().time_since_epoch().count();
        time_t elapsedTime = realtime_ - lastrealtime_;
        float playspeed=1;
        time_t deltareadtime =  elapsedTime * playspeed;

        readtime_ = lastreadtime_ + deltareadtime;
        samplesofdelatT[count_% 1000] = elapsedTime;

	// ICV_LOG_INFO << "readtime_:" << readtime_ << ", Maxtimeinfile:" << Maxtimeinfile_ 
	// 	<< ", Mintimeinfile" << Mintimeinfile_ << ", end_:" << end_;

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
                imshow("image repaly",read_img);
		        cv::waitKey(20);
                // ICV_LOG_INFO<<"finish imshow";
                img_data->setoutvalue(read_img);
                img_data->SetSourceTime(last_data->GetSourceTime());
                Send_Out(img_data,0);
            }
            /* play IMU data */

            if(last_data1->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data1, 1);
                Send_Out(last_data1,1);
                ICV_LOG_INFO<<"replay IMU";

            }
            /* play GPS data */
            if(last_data2->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data2, 2);
                Send_Out(last_data2,2);
                ICV_LOG_INFO<<"replay gps";
            }
            
            /* play Vel data */
            if(last_data3->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data3, 3);
                Send_Out(last_data3,3);
                ICV_LOG_INFO<<"replay vel";
            }
            
            /* play Odom data */
            if(last_data4->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data4, 4);
                Send_Out(last_data4,4);
                ICV_LOG_INFO<<"replay odom";
            }
            /* play LRR1 front data */
            if(last_data5->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data5, 5);
                Send_Out(last_data5,5);
                ICV_LOG_INFO<<"replay radar front";
            }
            /* play LRR2 rear data */
            if(last_data6->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data6, 6);
                Send_Out(last_data6,6);
                // ICV_LOG_INFO<<"replay radar rear";
            }
            /* play SRR1 front left data */
            if(last_data7->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data7, 7);
                Send_Out(last_data7,7);
                // ICV_LOG_INFO<<"replay radar side 1";
            }
            /* play SRR2 front right data */
            if(last_data8->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data8, 8);
                Send_Out(last_data8,8);
                // ICV_LOG_INFO<<"replay radar side 2";
            }
            /* play SRR3 rear left data */
            if(last_data9->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data9, 9);
                Send_Out(last_data9,9);
                // ICV_LOG_INFO<<"replay radar side 3";
            }
            /* play SRR4 rear right data */
            if(last_data0->GetSourceTime() < readtime_) 
            {
                _recorder->PlayNext(last_data0, 10);
                Send_Out(last_data0,10);
                // ICV_LOG_INFO<<"replay radar side 4";
            }
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
    vector<string> typenames_;
    int count_=0;
    float samplesofdelatT[1000];
    icvMsgpackRecorder* _recorder;
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
    icvsrr* last_data0 ;
    data::icvInt64Data* tempdata1;
    boost::filesystem::path image_path_,image_filename;
    cv::Mat read_img;
    };
ICV_REGISTER_FUNCTION(Replay)

}
}  


#endif



