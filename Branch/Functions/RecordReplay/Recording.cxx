
#ifndef _Recording_H
#define _Recording_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"
#include "OpenICV/Basis/icvMsgpackRecorder.h"
#include "OpenICV/Basis/icvMsgpackDeltaRecorder.h"
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
#include "OpenICV/Core/icvNodeManager.h"
#include "OpenICV/structure/icvPointCloudData.hxx"
// #include "OpenICV/structure/structureFusionData.h"
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



using namespace icv;
using namespace core;
using namespace std;
using namespace msgpack ;
using namespace cv;
ICV_CONSTEXPR char KEY_OUTPUT_FILEPATH[] = "path";
ICV_CONSTEXPR char KEY_RECORD_RANGE[] = "record range";
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
          if (_information.Contains(KEY_RECORD_RANGE))
        recordrange_ = _information.GetDecimal(KEY_RECORD_RANGE);
        else  recordrange_=1;
       

        _recorder = new icvMsgpackDeltaRecorder("", 0);

        mkdir(folder_path.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
     
       
        _recorder->setRecordpath(folder_path);
        global_path=boost::filesystem::path(folder_path);
    

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
       // count_++ ;
       if (read_input_flag){
       PublisherList=get_node()->GetMonitor()->GetAllPublisher();
     
       starttime_=icvTime::time_s();
       for (int i=0;i<PublisherList.size();i++){
           lasttimestamp.emplace(PublisherList[i]->getname(),0);
           currtimestamp.emplace(PublisherList[i]->getname(),0);

       }
         read_input_flag=false;
       }
       if(icvTime::time_s()-starttime_<recordrange_*60)
        {
       for (int i=0;i<PublisherList.size();i++){
           icvPublisher *pub_temp=PublisherList[i];
           bool datatypecheck=false;
           
           icv::core::icvDataObject* pub_buff= pub_temp->RequireDataObject();
        

            if(pub_buff) 
            {  
                currtimestamp.at(pub_temp->getname())=pub_buff->GetSourceTime();
                if(currtimestamp.at(pub_temp->getname())-lasttimestamp.at(pub_temp->getname())>0){
                //cout<<"new time: "<<pub_buff->GetSourceTime()<<"old time: "<<lasttime<<endl;
                lasttimestamp.at(pub_temp->getname())=currtimestamp.at(pub_temp->getname());
             
                string record_name=pub_temp->getname().substr(pub_temp->getname().find_last_of('.')+1,pub_temp->getname().length());
                _recorder->Record(pub_buff,record_name);
                //cout<<"sub in the name of: "<<record_name<<endl;
                reco_count++;
                //    if(record_name=="first_cam")
                //             {
                //                 imagefolderpath_=global_path/record_name;
                //                 mkdir(imagefolderpath_.string().c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
                //                 //data::icvCvMatData cam_msg;
                //                 //icvSubscribe("first_cam",&cam_msg);
                //                 //cv::Mat temp_mat= cam_msg.getvalue();
                //                 cv::Mat temp_mat= pub_buff->As<data::icvCvMatData>().getvalue();
                //                 //cout<<temp_mat.total()<<endl;
                //                 imshow("record picture",temp_mat);
                //                 waitKey(1);
                //                 //string tem_S=temp_mat.getImageName();
                //                 string tem_S=pub_buff->As<data::icvCvMatData>().getImageName();
                //                 //string tem_S=to_string(count_);
                //                 //cout<<"After transmition is "<<tem_S<<endl;
                //                 string file_name=(imagefolderpath_/tem_S).string()+".jpg";
                //                 //cout<<file_name<<endl;   
                //                 // std::cout<<file_name<<std::endl; 
                //                 cv::imwrite(file_name, temp_mat);

                //             }
                    if(record_name=="pandar_points"||record_name=="robosense_points")
                            {
                                pointcloud_path_=global_path/record_name;
                                mkdir(pointcloud_path_.string().c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
                                //data::icvCvMatData cam_msg;
                                //icvSubscribe("first_cam",&cam_msg);
                                //cv::Mat temp_mat= cam_msg.getvalue();
                                pcl::PointCloud<pcl::PointXYZI> temp_mat= pub_buff->As<data::icvPointCloudData<pcl::PointXYZI>>().getvalue();
                                //string tem_S=temp_mat.getImageName();
                                string tem_S=pub_buff->As<data::icvPointCloudData<pcl::PointXYZI>>().getPointCloudName();
                                //string tem_S=to_string(count_);
                                //cout<<"After transmition is "<<tem_S<<endl;
                                string file_name=(pointcloud_path_/tem_S).string()+".pcd";
                                //cout<<file_name<<endl;   
                                // std::cout<<file_name<<std::endl; 
                                pcl::io::savePCDFileBinaryCompressed (file_name,temp_mat);

                            }
                }
            }
            else {  ICV_LOG_WARN<<"RECORD DATA EMPTY"; }

           }
        }
        else
        {
             if(!fileclosed)
                    {
                        ICV_LOG_INFO << "Record how many data points: "<<reco_count;
                        this->~Recording();
                        _recorder->closefile();
                        fileclosed=true;
                    }
        }
        
       
 
        
    }
     
        
      
            
        
    
private:

int reco_count=0;
icv::icvMsgpackDeltaRecorder* _recorder;
//icvSemaphore* timesync_;


icv_map<std::string, time_t> lasttimestamp;
icv_map<std::string, time_t > currtimestamp;
icv_map<std::string, icvDataObject*> * alldata_=new icv_map<string, icvDataObject*>();

time_t starttime_, lasttime=0;
double recordrange_;

bool read_input_flag=true;


bool fileclosed=false;
bool first_=true;
std::vector<icvPublisher*> PublisherList;
string folder_path;
boost::filesystem::path imagefolderpath_, global_path, pointcloud_path_;
};

ICV_REGISTER_FUNCTION(Recording)


#endif  //
