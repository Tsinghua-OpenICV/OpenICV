
#ifndef _Recording_H
#define _Recording_H

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
#include <sys/stat.h>
#include <sys/types.h>
#include <opencv2/imgcodecs.hpp>

#include "OpenICV/Basis/icvStructureData.hxx"
#include "OpenICV/structure/structureCanTuan.h"
#include "OpenICV/structure/structureVehinfo.h"
#include "OpenICV/structure/structure_gps.h"
#include "OpenICV/structure/structureIMU.h"
#include "OpenICV/structure/structure_gps.h"
#include "OpenICV/structure/RadarRecordReplay.h"

// #include "OpenICV/structure/structureFusionData.h"
using namespace icv;
using namespace core;
using namespace std;
using namespace msgpack ;
ICV_CONSTEXPR char KEY_OUTPUT_FILEPATH[] = "path";

typedef data::icvStructureData<Imu> icvImu;
typedef data::icvStructureData<RadarLongOut>    icvlrr ;
typedef data::icvStructureData<RadarSideOut>    icvsrr ;
typedef data::icvStructureData<NavSatFix>    icvNavSatFix;
typedef data::icvStructureData<TwistWithCovarianceStamped>    icvTwistWithCovarianceStamped;
typedef data::icvStructureData<Odometry>    icvOdometry;

class Recording_old : public icvFunction
{
public:

    Recording_old(icv_shared_ptr<const icvMetaData> info) : icvFunction(info) 
    {
        recordrange_=3;

        timesync_=new icvSemaphore(1);
        stringstream ss;
            
        time_t now_time=icvTime::time_s();
        struct tm *local;
        local=localtime(&now_time);
        ICV_LOG_TRACE<<local->tm_year+1900<<"_"<<local->tm_mon<<"_"<<local->tm_mday<<"_"<<local->tm_hour<<"_"<<local->tm_min;

        ss<<local->tm_year+1900<<"_"<<local->tm_mon+1<<"_"<<local->tm_mday<<"_"<<local->tm_hour<<"_"<<local->tm_min;
        
        if (_information.Contains(KEY_OUTPUT_FILEPATH))
        folder_path = _information.GetString(KEY_OUTPUT_FILEPATH)+ss.str();
        else  folder_path=ss.str();

        _recorder = new icvMsgpackRecorder_old("", 0);

        mkdir(folder_path.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);

        _recorder->setRecordpath(folder_path);
        imagefolderpath_=boost::filesystem::path(folder_path);
        imagefolderpath_=imagefolderpath_/"image";
        mkdir(imagefolderpath_.string().c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
        // printf("%s\n",imagefolderpath_.c_str());
        cam_Sub=Register_Sub("first_cam");
        readdata=new icv::data::icvCvMatData();
        icv_thread loop(bind(&Recording_old::InnerLoop, this));	
	}
    Recording_old() : Recording_old(nullptr) {	
	
	}

    ~Recording_old() {
        _recorder->closefile();	
	}
    void InnerLoop()
    {
        while(true)
        {
        timesync_->Release();
        usleep(500);
        }
    }


    virtual void Execute()
    {
        count_++ ;
        timesync_->Lock();   
        
        icvSubscribe("first_cam",readdata);
        /* camera data */ 
        if(readdata->is_not_empty())
        {
            ICV_LOG_INFO<<"record image: "<<count_;
            
            currtimestamp[0]=readdata->GetSourceTime();

            if(first_)
            {	
                lasttimestamp[0]=currtimestamp[0];
                if (!start_)
                {
                    starttime_=icvTime::time_s();
                }
                start_=true;
                first_=false;
            }
            else
            {
                if(icvTime::time_s()-starttime_<recordrange_*60)
                {
                    if((currtimestamp[0]-lasttimestamp[0])>0)//to check new data
                    {
                        reco_count++;
                        _recorder->Record(readdata, 0);
                        cv::Mat temp_mat= readdata->getvalue();
                        cv::imshow("Source Image",temp_mat);
                        cv::waitKey(1);
                        string tem_S=readdata->getImageName();
                        string file_name=(imagefolderpath_/tem_S).string()+".jpg";   
                        // std::cout<<file_name<<std::endl; 
                        cv::imwrite(file_name, temp_mat);
                        lasttimestamp[0]=currtimestamp[0];
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

        // /* IMU data */
        // if(is_not_empty(1))
        // {
        //      ICV_LOG_INFO<<"debug1";
        //     readdata1=read_Input<icvImu>(1).get_Ptr();
        //     currtimestamp[1]=read_Input<icvImu>(1).GetSourceTime();

        //     if(first1_)
        //     {	
        //         lasttimestamp[1]=currtimestamp[1];
        //         if (!start_)
        //         {
        //             starttime_=SyncClock::time_s();
        //         }
        //         start_=true;
        //         first1_=false;

        //     }
        //     else
        //     {
        //         if(SyncClock::time_s()-starttime_<recordrange_*60)
        //         {
        //             if((currtimestamp[1]-lasttimestamp[1])>0)//to check new data
        //             {
        //                 _recorder->Record(readdata1, 1);
        //                 Imu tmp_data , tmp_data2;
        //                 tmp_data = readdata1->getStructData();
        //                 // ICV_LOG_INFO<< "debug1: "<<readdata1->getStructData().linear_acceleration.x ;
        //                 std::stringstream buff ;
        //                 int len ;
        //                 msgpack::unpacked result ;
        //                 msgpack::pack(buff,tmp_data);

        //                 unpack(&result,buff.str().data(),buff.str().size());
        //                 object obj = result.get();
        //                 obj.convert(tmp_data2);
        //                 // ICV_LOG_INFO<<"debug2: "<<tmp_data2.linear_acceleration.x ;
        //                 lasttimestamp[1]=currtimestamp[1];
        //             }
                    
        //         }
        //         else 
        //         {
        //             if(!fileclosed)
        //             {
        //                 _recorder->closefile();
        //                 fileclosed=true;
        //             }
        //         }
        //     }
        // }

        // /* GPS data */
        // if(is_not_empty(2))
        // {
        //      ICV_LOG_INFO<<"debug2";
        //     readdata2=read_Input<icvNavSatFix>(2).get_Ptr();
        //     currtimestamp[2]=read_Input<icvNavSatFix>(2).GetSourceTime();

        //     if(first2_)
        //     {	
        //         lasttimestamp[2]=currtimestamp[2];
        //         if (!start_)
        //         {
        //             starttime_=SyncClock::time_s();
        //         }
        //         start_=true;
        //         first2_=false;

        //     }
        //     else
        //     {
        //         if(SyncClock::time_s()-starttime_<recordrange_*60)
        //         {
        //             if((currtimestamp[2]-lasttimestamp[2])>0)//to check new data
        //             {
        //                 _recorder->Record(readdata2, 2);
        //                 lasttimestamp[2]=currtimestamp[2];
        //             }
                    
        //         }
        //         else 
        //         {
        //             if(!fileclosed)
        //             {
        //                 _recorder->closefile();
        //                 fileclosed=true;
        //             }
        //         }
        //     }
        // }
        // /* Vel data */
        // if(is_not_empty(3))
        // {
        //      ICV_LOG_INFO<<"debug3";
        //     readdata3=read_Input<icvTwistWithCovarianceStamped>(3).get_Ptr();
        //     currtimestamp[3]=read_Input<icvTwistWithCovarianceStamped>(3).GetSourceTime();

        //     if(first3_)
        //     {	
        //         lasttimestamp[3]=currtimestamp[3];
        //         if (!start_)
        //         {
        //             starttime_=SyncClock::time_s();
        //         }
        //         start_=true;
        //         first3_=false;

        //     }
        //     else
        //     {
        //         if(SyncClock::time_s()-starttime_<recordrange_*60)
        //         {
        //             if((currtimestamp[3]-lasttimestamp[3])>0)//to check new data
        //             {
        //                 _recorder->Record(readdata3, 3);
        //                 lasttimestamp[3]=currtimestamp[3];
        //             }
                    
        //         }
        //         else 
        //         {
        //             if(!fileclosed)
        //             {
        //                 _recorder->closefile();
        //                 fileclosed=true;
        //             }
        //         }
        //     }
        // }
        // /* Odom data */
        // if(is_not_empty(4))
        // {
        //      ICV_LOG_INFO<<"debug4";
        //     readdata4=read_Input<icvOdometry>(4).get_Ptr();
        //     currtimestamp[4]=read_Input<icvOdometry>(4).GetSourceTime();

        //     if(first4_)
        //     {	
        //         lasttimestamp[4]=currtimestamp[4];
        //         if (!start_)
        //         {
        //             starttime_=SyncClock::time_s();
        //         }
        //         start_=true;
        //         first4_=false;

        //     }
        //     else
        //     {
        //         if(SyncClock::time_s()-starttime_<recordrange_*60)
        //         {
        //             if((currtimestamp[4]-lasttimestamp[4])>0)//to check new data
        //             {
        //                 _recorder->Record(readdata4, 4);
        //                 lasttimestamp[4]=currtimestamp[4];
        //             }
                    
        //         }
        //         else 
        //         {
        //             if(!fileclosed)
        //             {
        //                 _recorder->closefile();
        //                 fileclosed=true;
        //             }
        //         }
        //     }
        // }
        
        // /* LRR1 front */
        // if(is_not_empty(5))
        // {
        //      ICV_LOG_INFO<<"debug5";
        //     readdata5=read_Input<icvlrr>(5).get_Ptr();
        //     currtimestamp[5]=read_Input<icvlrr>(5).GetSourceTime();

        //     if(first5_)
        //     {	
        //         lasttimestamp[5]=currtimestamp[5];
        //         if (!start_)
        //         {
        //             starttime_=SyncClock::time_s();
        //         }
        //         start_=true;
        //         first5_=false;

        //     }
        //     else
        //     {
        //         if(SyncClock::time_s()-starttime_<recordrange_*60)
        //         {
        //             if((currtimestamp[5]-lasttimestamp[5])>0)//to check new data
        //             {
        //                 _recorder->Record(readdata5, 5);
        //                 lasttimestamp[5]=currtimestamp[5];
        //             }
                    
        //         }
        //         else 
        //         {
        //             if(!fileclosed)
        //             {
        //                 _recorder->closefile();
        //                 fileclosed=true;
        //             }
        //         }
        //     }
        // }
        // /* LRR2 rear */
        // if(is_not_empty(6))
        // {
        //      ICV_LOG_INFO<<"debug6";
        //     readdata6=read_Input<icvlrr>(6).get_Ptr();
        //     currtimestamp[6]=read_Input<icvlrr>(6).GetSourceTime();

        //     if(first6_)
        //     {	
        //         lasttimestamp[6]=currtimestamp[6];
        //         if (!start_)
        //         {
        //             starttime_=SyncClock::time_s();
        //         }
        //         start_=true;
        //         first6_=false;

        //     }
        //     else
        //     {
        //         if(SyncClock::time_s()-starttime_<recordrange_*60)
        //         {
        //             if((currtimestamp[6]-lasttimestamp[6])>0)//to check new data
        //             {
        //                 _recorder->Record(readdata6, 6);
        //                 lasttimestamp[6]=currtimestamp[6];
        //             }
                    
        //         }
        //         else 
        //         {
        //             if(!fileclosed)
        //             {
        //                 _recorder->closefile();
        //                 fileclosed=true;
        //             }
        //         }
        //     }
        // }

        // /* SRR1 front left */
        // if(is_not_empty(7))
        // {
        //      ICV_LOG_INFO<<"debug7";
        //     readdata7=read_Input<icvsrr>(7).get_Ptr();
        //     currtimestamp[7]=read_Input<icvsrr>(7).GetSourceTime();

        //     if(first7_)
        //     {	
        //         lasttimestamp[7]=currtimestamp[7];
        //         if (!start_)
        //         {
        //             starttime_=SyncClock::time_s();
        //         }
        //         start_=true;
        //         first7_=false;

        //     }
        //     else
        //     {
        //         if(SyncClock::time_s()-starttime_<recordrange_*60)
        //         {
        //             if((currtimestamp[7]-lasttimestamp[7])>0)//to check new data
        //             {
        //                 _recorder->Record(readdata7, 7);
        //                 lasttimestamp[7]=currtimestamp[7];
        //             }
                    
        //         }
        //         else 
        //         {
        //             if(!fileclosed)
        //             {
        //                 _recorder->closefile();
        //                 fileclosed=true;
        //             }
        //         }
        //     }
        // }

        // /* SRR2 front right */
        // if(is_not_empty(8))
        // {
        //      ICV_LOG_INFO<<"debug8";
        //     readdata8=read_Input<icvsrr>(8).get_Ptr();
        //     currtimestamp[8]=read_Input<icvsrr>(8).GetSourceTime();

        //     if(first8_)
        //     {	
        //         lasttimestamp[8]=currtimestamp[8];
        //         if (!start_)
        //         {
        //             starttime_=SyncClock::time_s();
        //         }
        //         start_=true;
        //         first8_=false;

        //     }
        //     else
        //     {
        //         if(SyncClock::time_s()-starttime_<recordrange_*60)
        //         {
        //             if((currtimestamp[8]-lasttimestamp[8])>0)//to check new data
        //             {
        //                 _recorder->Record(readdata8, 8);
        //                 lasttimestamp[8]=currtimestamp[8];
        //             }
                    
        //         }
        //         else 
        //         {
        //             if(!fileclosed)
        //             {
        //                 _recorder->closefile();
        //                 fileclosed=true;
        //             }
        //         }
        //     }
        // }

        // /* SRR3 rear left */
        // if(is_not_empty(9))
        // {
        //      ICV_LOG_INFO<<"debug9";
        //     readdata9=read_Input<icvsrr>(9).get_Ptr();
        //     currtimestamp[9]=read_Input<icvsrr>(9).GetSourceTime();

        //     if(first9_)
        //     {	
        //         lasttimestamp[9]=currtimestamp[9];
        //         if (!start_)
        //         {
        //             starttime_=SyncClock::time_s();
        //         }
        //         start_=true;
        //         first9_=false;

        //     }
        //     else
        //     {
        //         if(SyncClock::time_s()-starttime_<recordrange_*60)
        //         {
        //             if((currtimestamp[9]-lasttimestamp[9])>0)//to check new data
        //             {
        //                 _recorder->Record(readdata9, 9);
        //                 lasttimestamp[9]=currtimestamp[9];
        //             }
                    
        //         }
        //         else 
        //         {
        //             if(!fileclosed)
        //             {
        //                 _recorder->closefile();
        //                 fileclosed=true;
        //             }
        //         }
        //     }
        // }

        // /* SRR4 rear right */
        // if(is_not_empty(10))
        // {
        //      ICV_LOG_INFO<<"debug";
        //     readdata0=read_Input<icvsrr>(10).get_Ptr();
        //     currtimestamp[10]=read_Input<icvsrr>(10).GetSourceTime();

        //     if(first0_)
        //     {	
        //         lasttimestamp[10]=currtimestamp[10];
        //         if (!start_)
        //         {
        //             starttime_=SyncClock::time_s();
        //         }
        //         start_=true;
        //         first0_=false;

        //     }
        //     else
        //     {
        //         if(SyncClock::time_s()-starttime_<recordrange_*60)
        //         {
        //             if((currtimestamp[10]-lasttimestamp[10])>0)//to check new data
        //             {
        //                 _recorder->Record(readdata0, 10);
        //                 lasttimestamp[10]=currtimestamp[10];
        //             }
                    
        //         }
        //         else 
        //         {
        //             if(!fileclosed)
        //             {
        //                 _recorder->closefile();
        //                 fileclosed=true;
        //             }
        //         }
        //     }
        // }
    }
private:

int input_count_;
int reco_count=0;
vector<string> typenames_;
int count_=0;
icv::icvMsgpackRecorder_old* _recorder;
icvSemaphore* timesync_;
time_t lasttimestamp[11];
time_t currtimestamp[11];
time_t starttime_;
double recordrange_;
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
icvsrr* readdata0 ;
icvSubscriber* cam_Sub;


bool fileclosed=false;

bool start_=false;

bool first_=true;
bool first1_=true;
bool first2_=true;
bool first3_=true;
bool first4_=true;
bool first5_=true;
bool first6_=true;
bool first7_=true;
bool first8_=true;
bool first9_=true;
bool first0_=true;

string folder_path;
boost::filesystem::path imagefolderpath_, image_file_path;
};

ICV_REGISTER_FUNCTION(Recording_old)


#endif  //
