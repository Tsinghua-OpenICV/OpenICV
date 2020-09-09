
#ifndef _Recording_H
#define _Recording_H

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
#include <sys/stat.h>
#include <sys/types.h>
#include <opencv2/imgcodecs.hpp>

#include "OpenICV/Basis/icvStructureData.hxx"
#include "OpenICV/structure/structureCanTuan.h"
#include "OpenICV/structure/structureVehinfo.h"
#include "OpenICV/structure/structure_gps.h"
#include "OpenICV/structure/structureIMU.h"
// #include "OpenICV/structure/structure_gps.h"
#include "OpenICV/structure/RadarRecordReplay.h"

// #include "OpenICV/structure/structureFusionData.h"
using namespace icv;
using namespace core;
using namespace std;
using namespace msgpack ;
using namespace cv;
ICV_CONSTEXPR char KEY_OUTPUT_FILEPATH[] = "path";

typedef data::icvStructureData<Imu> icvImu;
typedef data::icvStructureData<RadarLongOut>    icvlrr ;
typedef data::icvStructureData<RadarSideOut>    icvsrr ;
typedef data::icvStructureData<NavSatFix>    icvNavSatFix;
typedef data::icvStructureData<TwistWithCovarianceStamped>    icvTwistWithCovarianceStamped;
typedef data::icvStructureData<Odometry>    icvOdometry;

class Recording : public icvFunction
{
public:

    Recording(icv_shared_ptr<const icvMetaData> info) : icvFunction(info) 
    {
        //recordrange_= _information.GetDecimal("Dur_min");
        if (_information.Contains(KEY_OUTPUT_FILEPATH))
        folder_path = _information.GetString(KEY_OUTPUT_FILEPATH)+date_of_now();
        else  folder_path=date_of_now();
     
        _recorder = new _impl::icvMsgpackRecorder("", 0);

        mkdir(folder_path.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);

        _recorder->setRecordpath(folder_path);
        imagefolderpath_=boost::filesystem::path(folder_path);
        imagefolderpath_=imagefolderpath_/"image";
        mkdir(imagefolderpath_.string().c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
        // printf("%s\n",imagefolderpath_.c_str());
        cam_Sub=Register_Sub("first_cam");
        readdata=new icv::data::icvCvMatData();
        //cam_Sub->setname("first_cam");
        /*
        Register_Sub("IMU");
        Register_Sub("NavSat");
        Register_Sub("Twist");
        Register_Sub("Odometry");
        Register_Sub("radar_fr");
        Register_Sub("radar_re");
        Register_Sub("radar_lf");
        Register_Sub("radar_lr");
        Register_Sub("radar_rf");
        Register_Sub("radar_rr");
        */

	}
    Recording() : Recording(nullptr) {	
	
	}
    
    ~Recording() {
        _recorder->closefile();	
	}

    string date_of_now()
    {
        stringstream ss;
        time_t now_time=icvTime::time_s();
        struct tm *local;
        local=localtime(&now_time);
        //ICV_LOG_TRACE<<local->tm_year+1900<<"_"<<local->tm_mon+1<<"_"<<local->tm_mday<<"_"<<local->tm_hour<<"_"<<local->tm_min;

                   ss<<local->tm_year+1900<<"_"<<local->tm_mon+1<<"_"<<local->tm_mday<<"_"<<local->tm_hour<<"_"<<local->tm_min;
        return ss.str();


    }

    virtual void Execute() override
    {
        count_++ ;
        
        icvSubscribe("first_cam",readdata);
       
        
        alldata_->emplace(cam_Sub->getname(),readdata); 


        

        //cout<<"time to receive is "<<readdata->GetSourceTime()<<endl;
        /*
        alldata_->emplace(subs1->getname(),icvSubscribe<icv::data::icvCvMatData>("IMU"));
        alldata_->emplace(subs2->getname(),icvSubscribe<icv::data::icvCvMatData>("NavSat"));
        alldata_->emplace(subs3->getname(),icvSubscribe<icv::data::icvCvMatData>("Twist"));
        alldata_->emplace(subs4->getname(),icvSubscribe<icv::data::icvCvMatData>("Odometry"));
        alldata_->emplace(subs5->getname(),icvSubscribe<icv::data::icvCvMatData>("radar_fr"));
        alldata_->emplace(subs6->getname(),icvSubscribe<icv::data::icvCvMatData>("radar_re"));
        alldata_->emplace(subs7->getname(),icvSubscribe<icv::data::icvCvMatData>("radar_lf"));
        alldata_->emplace(subs8->getname(),icvSubscribe<icv::data::icvCvMatData>("radar_lr"));
        alldata_->emplace(subs9->getname(),icvSubscribe<icv::data::icvCvMatData>("radar_rf"));
        alldata_->emplace(subs10->getname(),icvSubscribe<icv::data::icvCvMatData>("radar_rr"));
        */
        // readdata1=icvSubscribe<icvImu>(subs1);
        // readdata2=icvSubscribe<icvNavSatFix>(subs2);
        // readdata3=icvSubscribe<icvTwistWithCovarianceStamped>(subs3);
        // readdata4=icvSubscribe<icvOdometry>(subs4);             
        // readdata5=icvSubscribe<icvlrr>(subs5);
        // readdata6=icvSubscribe<icvlrr>(subs6);
        // readdata7=icvSubscribe<icvsrr>(subs7);
        // readdata8=icvSubscribe<icvsrr>(subs8);
        // readdata9=icvSubscribe<icvsrr>(subs9);
        // readdata10=icvSubscribe<icvsrr>(subs10);

            if(count_>5)
            {	
                
                starttime_=icvTime::time_s();
                currtimestamp.at("first_cam") = readdata->GetSourceTime();
                //currtimestamp.at("first_cam")=readdata->GetSourceTime();
                //currtimestamp.insert(pair<int, string>(1, "student_one"));  
                
                /*
                currtimestamp.emplace(subs1->getname(),readdata1->GetSourceTime());
                currtimestamp.emplace(subs2->getname(),readdata2->GetSourceTime());
                currtimestamp.emplace(subs3->getname(),readdata3->GetSourceTime());
                currtimestamp.emplace(subs4->getname(),readdata4->GetSourceTime());
                currtimestamp.emplace(subs5->getname(),readdata5->GetSourceTime());
                currtimestamp.emplace(subs6->getname(),readdata6->GetSourceTime());
                currtimestamp.emplace(subs7->getname(),readdata7->GetSourceTime());
                currtimestamp.emplace(subs8->getname(),readdata8->GetSourceTime());
                currtimestamp.emplace(subs9->getname(),readdata9->GetSourceTime());
                currtimestamp.emplace(subs10->getname(),readdata10->GetSourceTime()); */

                //lasttimestamp->emplace(cam_Sub->getname(),1);
                /*
                lasttimestamp.emplace(subs1->getname(),1);
                lasttimestamp.emplace(subs2->getname(),2);
                lasttimestamp.emplace(subs3->getname(),3);
                lasttimestamp.emplace(subs4->getname(),4);
                lasttimestamp.emplace(subs5->getname(),5);
                lasttimestamp.emplace(subs6->getname(),6);
                lasttimestamp.emplace(subs7->getname(),7);
                lasttimestamp.emplace(subs8->getname(),8);
                lasttimestamp.emplace(subs9->getname(),9);
                lasttimestamp.emplace(subs10->getname(),10);
                */
                
            }
            if(count_>5)
                {
                if(icvTime::time_s()-starttime_<recordrange_*60)
                {
                    
                   
                    for(auto topicname: *(get_sub_names()))
                    {
                       //cout<<"The two time are "<<currtimestamp->at(topicname)<<" and "<<lasttimestamp->at(topicname)<<endl;
                        if((currtimestamp.at(topicname)-lasttimestamp.at(topicname))>0)
                        {
                        lasttimestamp.at(topicname)=currtimestamp.at(topicname);
                        _recorder->Record(alldata_->at(topicname), topicname);
                        
                            if(topicname=="first_cam")
                            {
                                
                                //data::icvCvMatData cam_msg;
                                //icvSubscribe("first_cam",&cam_msg);
                                //cv::Mat temp_mat= cam_msg.getvalue();
                                cv::Mat temp_mat= alldata_->at(topicname)->As<data::icvCvMatData>().getvalue();
                                //cout<<temp_mat.total()<<endl;
                                imshow("example",temp_mat);
                                waitKey(1);
                                //string tem_S=temp_mat.getImageName();
                                string tem_S=alldata_->at(topicname)->As<data::icvCvMatData>().getImageName();
                                //string tem_S=to_string(count_);
                                //cout<<"After transmition is "<<tem_S<<endl;
                                string file_name=(imagefolderpath_/tem_S).string()+".jpg";
                                //cout<<file_name<<endl;   
                                // std::cout<<file_name<<std::endl; 
                                cv::imwrite(file_name, temp_mat);

                            }


                        }




                    }
                    
                    
                }
                else 
                {
                    if(!fileclosed)
                    {
                        ICV_LOG_INFO << "Record how many pics:"<<reco_count;
                        _recorder->closefile();
                        fileclosed=true;
                    }
                }
            }
        
    }
     
        
      
            
        
    
private:

int input_count_;
int reco_count=0;
vector<string> typenames_;
int count_=0;
icv::_impl::icvMsgpackRecorder* _recorder;
//std::map<std::string, time_t> lasttimestamp;
//std::map<std::string, time_t > currtimestamp;

icv_map<std::string, time_t> lasttimestamp= {
                { "first_cam", 0}
              };
icv_map<std::string, time_t > currtimestamp={
                { "first_cam", 0}
              };
icv_map<std::string, icvDataObject*> * alldata_=new icv_map<string, icvDataObject*>();

time_t starttime_;
double recordrange_=3;
icv::data::icvCvMatData* readdata;
icvImu* readdata1 ;
icvNavSatFix* readdata2 ;
icvTwistWithCovarianceStamped* readdata3;
icvOdometry* readdata4;
icvlrr* readdata5 ;
icvlrr* readdata6 ;
icvsrr* readdata7 ;
icvsrr* readdata8 ;
icvsrr* readdata9 ;
icvsrr* readdata10 ;



bool fileclosed=false;
bool first_=true;

icvSubscriber* cam_Sub;
icvSubscriber* subs1;
icvSubscriber* subs2;
icvSubscriber* subs3;
icvSubscriber* subs4;
icvSubscriber* subs5;
icvSubscriber* subs6;
icvSubscriber* subs7;
icvSubscriber* subs8;
icvSubscriber* subs9;
icvSubscriber* subs10;
string folder_path;
boost::filesystem::path imagefolderpath_, image_file_path;
};

ICV_REGISTER_FUNCTION(Recording)


#endif  //
