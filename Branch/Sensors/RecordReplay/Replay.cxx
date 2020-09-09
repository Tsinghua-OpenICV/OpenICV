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


ICV_CONSTEXPR char KEY_OUTPUT_FILEPATH[] = "path";


namespace icv { namespace function
{
    using namespace std;
    using namespace icv::core;
    using namespace _impl;
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
	    
        std::string path;
        if (_information.Contains(KEY_OUTPUT_FILEPATH)){
            path = _information.GetString(KEY_OUTPUT_FILEPATH);
           }
        else ICV_LOG_FATAL<<"NO FILE PATH";
        



        image_path_=boost::filesystem::path(path);
        global_path=boost::filesystem::path(path);
      
        image_path_=image_path_.parent_path()/"image";
  
         _recorder = new _impl::icvMsgpackRecorder(boost::filesystem::path(path), 0);
        //_recorder = new _impl::icvMsgpackRecorder(boost::filesystem::system_complete(path), 0);
        //Mintimeinfile_=_recorder->getheader().MinTime;
        //Maxtimeinfile_=_recorder->getheader().MaxTime;
        Mintimeinfile_=1551151730209840;
        Maxtimeinfile_=1551151750545299;
        cout<<"max time is "<<Maxtimeinfile_<<" and min time is "<<Mintimeinfile_<<endl;


        topic_names.push_back("first_cam");
        /*
        topic_names.push_back("IMU");
        topic_names.push_back("NavSat");
        topic_names.push_back("Twist");
        topic_names.push_back("Odometry");
        topic_names.push_back("radar_fr");
        topic_names.push_back("radar_re");
        topic_names.push_back("radar_lf");
        topic_names.push_back("radar_lr");
        topic_names.push_back("radar_rf");
        topic_names.push_back("radar_rr");
        
        */
        alldata_->emplace(topic_names[0],new data::icvCvMatData());
        /*

        alldata_->emplace(topic_names[1],new icvImu());
        alldata_->emplace(topic_names[2],new icvNavSatFix());
        alldata_->emplace(topic_names[3],new icvTwistWithCovarianceStamped());
        alldata_->emplace(topic_names[4],new icvOdometry());
        alldata_->emplace(topic_names[5],new icvlrr());
        alldata_->emplace(topic_names[6],new icvlrr());
        alldata_->emplace(topic_names[7],new icvsrr());
        alldata_->emplace(topic_names[8],new icvsrr());
        alldata_->emplace(topic_names[9],new icvsrr());
        alldata_->emplace(topic_names[10],new icvsrr());

        */
        img_data = new data::icvCvMatData ();        
           
        
      

        lastrealtime_ = icvTime::time_us();
        lastreadtime_=Mintimeinfile_;

        // ICV_LOG_INFO<<"time min"<<Mintimeinfile_;	
        last_data->SetSourceTime(Mintimeinfile_);
        // cv::namedWindow("repaly1");	
        
	}
    
    void Execute() 
    {
  
        count_++ ;
        realtime_ = icvTime::time_us();
        time_t elapsedTime = realtime_ - lastrealtime_;
        float playspeed=1;
        time_t deltareadtime =  elapsedTime * playspeed;
        
        readtime_ = lastreadtime_ + deltareadtime;
      //  samplesofdelatT[count_% 1000] = elapsedTime;

	// ICV_LOG_INFO << "readtime_:" << readtime_ << ", Maxtimeinfile:" << Maxtimeinfile_ 
	// 	<< ", Mintimeinfile" << Mintimeinfile_ << ", end_:" << end_;
        if ((readtime_ <= Maxtimeinfile_) && (readtime_ >= Mintimeinfile_))
        {
            
          
            lastrealtime_ = realtime_ ;
            lastreadtime_ = readtime_ ; 
            /* play camera data */ 
            for(auto topic_:topic_names)
            {

             
                
            if(alldata_->at(topic_)->GetSourceTime() < readtime_) 
            { 
                  
                    _recorder->PlayNext(alldata_->at(topic_), topic_);
                   
                
                if(topic_=="first_cam")
                { 
                            
                
                            string pictoread=alldata_->at(topic_)->As<data::icvCvMatData>().getImageName()+".jpg";
                            image_filename=image_path_/pictoread;
                            // ICV_LOG_INFO<<"address"<<image_filename.string();
                            read_img=cv::imread(image_filename.string());
                            imshow("replay",read_img);
                            cv::waitKey(20);
                            // ICV_LOG_INFO<<"finish imshow";
                            img_data->setvalue(read_img);
                            img_data->SetSourceTime(alldata_->at(topic_)->GetSourceTime());
                            icvPublish(topic_,img_data);
                            
                           /*
                            Directory dir;
                            vector<string> fileNames = dir.GetListFiles(img_path, "*.jpg", false);
                            
                            for (int i = 0; i < fileNames.size(); i++)
                            {
                                string fileName = fileNames[i];
                                string fileFullName = img_path + fileName;
                                cout << "file name:" << fileName << endl;
                                cout << "file paht:" << fileFullName << endl << endl;
                        
                                //Image processing
                                Mat pScr;
                                pScr = imread(fileFullName, 1); 
                                imshow("picture", pScr);
                                waitKey(30);
                        
                        
                            }
                            */

                }

                else
                {
                     
                        _recorder->PlayNext(alldata_->at(topic_),topic_);
                        icvPublish(topic_,alldata_->at(topic_));

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
            }
        }
    }
    
    

    private:
    bool first_=true,end_=true;
    int output_count_;
    time_t lastreadtime_,lastrealtime_,readtime_,realtime_,Mintimeinfile_,Maxtimeinfile_;
    vector<string> typenames_;
    int count_=0;
    icv_map<std::string, icvDataObject*> * alldata_=new icv_map<string, icvDataObject*>();
    float samplesofdelatT[1000];
    vector<string> topic_names;
    icvMsgpackRecorder* _recorder;
    icvSemaphore* timesync_;
    data::icvCvMatData *last_data, *img_data;
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
    boost::filesystem::path image_path_,image_filename,global_path;
    cv::Mat read_img;
    };
ICV_REGISTER_FUNCTION(Replay)

}
}  


#endif



