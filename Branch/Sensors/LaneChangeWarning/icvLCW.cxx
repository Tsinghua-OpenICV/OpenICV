#ifndef _LCWFunction_H
#define _LCWFunction_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Basis/icvStructureData.hxx"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Basis/icvSemaphore.h"

#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"
#include "OpenICV/structure/icvCvMatData.hxx"

#include "OpenICV/structure/structureIMU.h"
#include "OpenICV/structure/structureFusionData.h"
#include "OpenICV/structure/structureLaneData.h"

#include <boost/thread/thread.hpp>
#include <iostream>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>   
#include <opencv2/highgui/highgui.hpp>   
#include <opencv2/imgproc/imgproc.hpp>   
#include <opencv2/features2d/features2d.hpp>  

#include "common.h"
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace icv;
using namespace core;
using namespace std;

typedef data::icvStructureData<TrackArray> icvfusion ;
typedef data::icvStructureData<ld_Frame> icvlane ;

class LCWFunction : public icvFunction
{
public:
    LCWFunction(icv_shared_ptr<const icvMetaData> info) : icvFunction(info) 
    {
        lcw_visual.create(300, 360, CV_8UC3);
        lcw_visual.setTo(255);
        cv::circle(lcw_visual, cv::Point2f(320,30),10,cv::Scalar(0,255,0),-1,8,0);
        cv::rectangle(lcw_visual,cv::Point2f(150,140),cv::Point2f(170,160),cv::Scalar(255,0,0),-1,8,0);
        for (int i=0;i<5;i++)
        {
            pre_warining_level[i]=1;
            warning_keep_num[i]=0;
            warning_times[i]=0;
        }
        warning_keep_threshold = 200 ;
        warning_times_threshold = 0 ;
        timesync_=new icvSemaphore(1);
        icv_thread loop(bind(&LCWFunction::InnerLoop, this));
        Register_Pub("lcw_image");
		Register_Sub("lanedata");
		Register_Sub("fusion_output");
	}

    /* 接收融合信息 */
    void callback_fusion_result(const TrackArray msg)
    {
        OBSVector3d speed;
        OBSVector3d acc;
        float yaw;
        speed._x = msg.self_velocity.x;
        speed._y = msg.self_velocity.y;
        speed._z = msg.self_velocity.z;
        acc._x = msg.self_acceleration.x;
        acc._y = msg.self_acceleration.y;
        acc._z = msg.self_acceleration.z;
        yaw = msg.imu_pose.z;

        CCommonUnit.CarParam_set(speed,yaw,acc);
        CLcwUnit.CarParam_set(speed,yaw, acc);
        CCommonUnit.Update_Target_Status(msg);
    }

    /* 接收车道线检测信息 */
    void callback_lane_data(const ld_Frame msg)
    {
        CCommonUnit.LaneParam_set(msg);
    }

    /* 换道危险评估与预警 */
    void lane_danger_check()
    {

        //选择TTC最小目标
        Select_Min_TTC(CCommonUnit._Vec_TargetsInfo_Classfied[0]);
        Select_Min_TTC(CCommonUnit._Vec_TargetsInfo_Classfied[1]);
        Select_Min_TTC(CCommonUnit._Vec_TargetsInfo_Classfied[2]);
        Select_Min_TTC(CCommonUnit._Vec_TargetsInfo_Classfied[3]);
        Select_Min_TTC(CCommonUnit._Vec_TargetsInfo_Classfied[4]);
        Select_Min_TTC(CCommonUnit._Vec_TargetsInfo_Classfied[5]);

        // 分析换道危险
        CLcwUnit.TargetLaneFront_SafeDist(CCommonUnit._Vec_TargetsInfo_Classfied[1]);
        CLcwUnit.TargetLaneRear_SafeDist(CCommonUnit._Vec_TargetsInfo_Classfied[2]);
        CLcwUnit.TargetLaneFront_SafeDist(CCommonUnit._Vec_TargetsInfo_Classfied[3]);
        CLcwUnit.TargetLaneRear_SafeDist(CCommonUnit._Vec_TargetsInfo_Classfied[4]);
    
        //判断有无车道，无车道线则标记该侧不可换道
        std::vector<float> a;
        for (int ii=0;ii<5;ii++){
            a.push_back(1);
        }
        if (CCommonUnit.left_lane_flag==false){
            a[1]=0 ;
            a[2]=0 ;
        }

        if (CCommonUnit.right_lane_flag==false){
            a[3]=0;
            a[4]=0;
        }
 
        //初始化危险目标容器
        std::vector<TARGETS_INFO> temp;   
        TARGETS_INFO tmp_target ;
        temp.push_back(tmp_target);
        temp.push_back(tmp_target);
        temp.push_back(tmp_target);
        temp.push_back(tmp_target);
        temp.push_back(tmp_target);

        int index ;
        //输出危险目标状态信息
        for(int jj=0;jj<5;jj++){
            // 存在危险目标，输出危险目标位置、状态
            if(CCommonUnit._Vec_TargetsInfo_Classfied[jj]._warning_flag==1){

                index = CCommonUnit._Vec_TargetsInfo_Classfied[jj]._warning_id;
                if(jj==0){
                std::cout<<"××××danger front id×××:"<<index<<"×××obs position××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._x<<","<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._y<<std::endl;	
                std::cout<<"×××obs velocity××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._x<<","<<CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._y<<std::endl;
                a[0]=(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist/CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._safe_dist;
                warning_times[0] = warning_times[0]+1;
                if (warning_times[0]>warning_times_threshold)
                {
                    a[0]=0;
                    tmp_target._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);
                }else
                {
                    a[0]=1 ;
                }
                pre_warining_level[0] = a[0] ;
                temp[0]._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);
                }

                else if (jj==1 && CCommonUnit.left_lane_flag){
                std::cout<<"××××danger left front id×××:"<<index<<"×××obs position××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._x<<","<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._y<<std::endl;	
                std::cout<<"×××obs velocity××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._x<<","<<CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._y<<std::endl;
                a[1]=(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist/CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._safe_dist;
                
                warning_times[1] = warning_times[1]+1;
                if (warning_times[1]>warning_times_threshold)
                {
                    a[1]=0;
                    
                    tmp_target._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);
                }else
                {
                    a[1]=1 ;
                }
                pre_warining_level[1] = a[1];
                temp[1]._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);
                }

                else if (jj==2 && CCommonUnit.left_lane_flag){
                std::cout<<"××××danger left back id×××:"<<index<<"×××obs position××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._x<<","<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._y<<std::endl;	
                std::cout<<"×××obs velocity××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._x<<", self vel"<<CCommonUnit._VehSpeed._x \
                <<" ego acc: "<< CCommonUnit._VehAcceleration._x << " target acc: "<<CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._acceleration._x <<std::endl;
                a[2] = (float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist/CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._safe_dist;
                
                warning_times[2] = warning_times[2]+1;
                if (warning_times[2]>warning_times_threshold)
                {
                    a[2]=0;         
                    tmp_target._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);
                }else
                {
                    a[2]=1 ;
                }
                pre_warining_level[2]=a[2];
                temp[2]._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);
                }      

                else if (jj==3 && CCommonUnit.right_lane_flag){
                std::cout<<"××××danger right front id×××:"<<index<<"×××obs position××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._x<<","<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._y<<std::endl;
                std::cout<<"×××obs velocity××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._x<<","<<CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._y<<std::endl;
                
                a[3]=(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist/CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._safe_dist;
                warning_times[3] = warning_times[3]+1;
                if (warning_times[3]>warning_times_threshold)
                {
                    a[3]=0;
                    
                    tmp_target._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);
                }else
                {
                    a[3]=1 ;
                }  
                pre_warining_level[3]=a[3];
                temp[3]._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);
                } 

                else if (jj==4 && CCommonUnit.right_lane_flag){
                std::cout<<"××××danger right back id×××:"<<index<<"×××obs position××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._x<<","<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._y<<std::endl;	
                std::cout<<"×××obs velocity××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._x<<","<<CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._y<<std::endl;      
                a[4]=(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist/CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._safe_dist;
                
                warning_times[4] = warning_times[4]+1;
                if (warning_times[4]>warning_times_threshold)
                {
                    a[4]=0; 
                    tmp_target._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);
                }else
                {
                    a[4]=1 ;
                }
                pre_warining_level[4]=a[4];
                temp[4]._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);
                }
                
            }else 
            { // 侧后没有目标&&当前帧未检测到&&上一帧检测到 持续报警
                if (jj==0 && a[0]==1 && pre_warining_level[0]!=1){
                    a[0] = pre_warining_level[0];
                    warning_keep_num[0]++;
                    if( warning_keep_num[0] > warning_keep_threshold)
                    {
                    pre_warining_level[0] = 1;
                    warning_keep_num[0] = 0;
                    }
                }
                else if (jj==1 && a[1]==1 && pre_warining_level[1]!=1){
                    a[1] = pre_warining_level[1];
                    warning_keep_num[1]++;
                    if( warning_keep_num[1] > warning_keep_threshold)
                    {
                    pre_warining_level[1] = 1;
                    warning_keep_num[1] = 0;
                    }
                }
                else if (jj==2 && a[2]==1 && pre_warining_level[2]!=1){
                    a[2] = pre_warining_level[2];
                    warning_keep_num[2]++;
                    if( warning_keep_num[2] > warning_keep_threshold)
                    {
                    pre_warining_level[2] = 1;
                    warning_keep_num[2] = 0;
                    }
                }
                else if (jj==3 && a[3]==1 && pre_warining_level[3]!=1){
                    a[3] = pre_warining_level[3];
                    warning_keep_num[3]++;
                    if( warning_keep_num[3] > warning_keep_threshold)
                    {
                    pre_warining_level[3] = 1;
                    warning_keep_num[3] = 0;
                    }
                }
                else if (jj==4 && a[4]==1 && pre_warining_level[4]!=1){
                    a[4] = pre_warining_level[4];
                    warning_keep_num[4]++;
                    if( warning_keep_num[4] > warning_keep_threshold)
                    {
                    pre_warining_level[4] = 1;
                    warning_keep_num[4] = 0;
                    }
                }
            }

            }
        }

        /* 基于OpenCV的显示界面，车道用矩形框表示，危险车道变红，目标车用圆点表示 */
        void WarningGui(std::vector<float> warning_flag, std::vector<TARGETS_INFO> danger_targets){
 
            cv::Mat tmp_visual = lcw_visual.clone();

            std::string str="Lane Change Warning" ;
            cv::putText(tmp_visual, str, cv::Point2f(20,15),cv::FONT_HERSHEY_DUPLEX,0.5,CV_RGB(0,0,255),0.1,8,0);
            cv::rectangle(tmp_visual,cv::Point2f(120,30),cv::Point2f(200,270),cv::Scalar(0,255,0),2,8,0);

            if (warning_flag[0]<1){
                cv::rectangle(tmp_visual,cv::Point2f(120,30),cv::Point2f(200,270),cv::Scalar(0,0,255),2,8,0);
                std::string str = "Danger Front" ;
                cv::putText(tmp_visual, str, cv::Point2f(120,290),cv::FONT_HERSHEY_DUPLEX,0.3,CV_RGB(255,0,0),0.2,8,0);
            }

            if (CCommonUnit.left_lane_flag){
            cv::rectangle(tmp_visual,cv::Point2f(40,30),cv::Point2f(120,270),cv::Scalar(0,255,0),2,8,0);
            if (warning_flag[1]<1 || warning_flag[2]<1 ){
                cv::rectangle(tmp_visual,cv::Point2f(40,30),cv::Point2f(120,270),cv::Scalar(0,0,255),2,8,0);
                std::string str = "Danger Left" ;
                cv::putText(tmp_visual, str, cv::Point2f(40,290),cv::FONT_HERSHEY_DUPLEX,0.3,CV_RGB(255,0,0),0.2,8,0);
            }
            }else {
                std::string str = "NO LEFT LANE" ;
                cv::putText(tmp_visual, str, cv::Point2f(40,290),cv::FONT_HERSHEY_DUPLEX,0.3,CV_RGB(255,0,0),0.2,8,0);
            }
            
            if (CCommonUnit.right_lane_flag){
            cv::rectangle(tmp_visual,cv::Point2f(200,30),cv::Point2f(280,270),cv::Scalar(0,255,0),2,8,0);
            if (warning_flag[3]<1 || warning_flag[4]<1 ){
                cv::rectangle(tmp_visual,cv::Point2f(200,30),cv::Point2f(280,270),cv::Scalar(0,0,255),2,8,0);
                std::string str = "Danger Right" ;
                cv::putText(tmp_visual, str, cv::Point2f(200,290),cv::FONT_HERSHEY_DUPLEX,0.3,CV_RGB(255,0,0),0.2,8,0);
            }
            }else {
                std::string str = "NO RIGHT LANE" ;
                cv::putText(tmp_visual, str, cv::Point2f(200,290),cv::FONT_HERSHEY_DUPLEX,0.3,CV_RGB(255,0,0),0.2,8,0);
            }

            for (int j=0;j<5;j++){
                if (danger_targets[j]._target_info.size()>0)
                {
                    float x ;
                    float y ;

                    if (danger_targets[j]._target_info[0]._obs_position._x>20){
                        x=30 ;
                    }else if (danger_targets[j]._target_info[0]._obs_position._x<-20){
                        x= 270 ;
                    } else {
                        x=150-6*danger_targets[j]._target_info[0]._obs_position._x ;
                    }

                    float l_dist ;
                    float r_dist ;

                    if(CCommonUnit._LaneDepValue < 0)
                    {
                        l_dist = CCommonUnit._LaneWidth/2  + CCommonUnit._LaneDepValue;
                        r_dist = CCommonUnit._LaneWidth - l_dist ;
                    }else{
                        r_dist = CCommonUnit._LaneWidth/2 - CCommonUnit._LaneDepValue;
                        l_dist = CCommonUnit._LaneWidth - r_dist ;
                    }

                    if (danger_targets[j]._target_info[0]._obs_position._y> l_dist){
                        y=80 ;
                    }else if (danger_targets[j]._target_info[0]._obs_position._y <-r_dist){
                        y=240 ;
                    }else {
                        y=160 ;
                        x=40;
                    }

                    if (warning_flag[j]<1){
                        if (danger_targets[j]._target_info[0]._dist < 3)
                        {
                            std::cout<< "Danger Vehicle in Blind Area !"<< std::endl ;
                            cv::circle(tmp_visual, cv::Point2f(y,x),10,cv::Scalar(0,0,255),-1,8,0);
                        }
                        else if (danger_targets[j]._target_info[0]._dist > 3)
                        {
                            cv::circle(tmp_visual, cv::Point2f(y,x),5,cv::Scalar(0,0,255),2,8,0);
                        }
                    }else
                    {
                        if (danger_targets[j]._target_info[0]._dist < 3)
                        {
                            std::cout<< "Danger Vehicle in Blind Area !"<< std::endl ;
                            cv::circle(tmp_visual, cv::Point2f(y,x),10,cv::Scalar(255,0,0),-1,8,0);
                        }
                        else if (danger_targets[j]._target_info[0]._dist > 3)
                        {
                            cv::circle(tmp_visual, cv::Point2f(y,x),5,cv::Scalar(255,0,0),2,8,0);
                        }
                    }
                    
                }
                
            }
            cv::imshow("tmp_visual", tmp_visual) ;
            cv::waitKey(1);
        }
        virtual void Execute() override
        {
            timesync_->Lock();

        icvlane lanedata;
        icvSubscribe("lanedata",&lanedata);
		
        icvfusion fusiondata;
        icvSubscribe("fusion_output",&fusiondata);
            
        if (lanedata.is_not_empty()&&fusiondata.is_not_empty())
            {
                callback_fusion_result(fusion_data);
                callback_lane_data(lane_data);
                lane_danger_check(); 
                //WarningGui(flag_warning,Warning_object);
            }
        }

private:
void InnerLoop()
{
    while(true) 
    {
        timesync_->Release();
        usleep(50000);
    }
}
icvSemaphore* timesync_;
CCommon CCommonUnit;
CFunc_lcw CLcwUnit;
double pre_warining_level[5];
int warning_keep_num[5];
int warning_keep_threshold;
int warning_times[5];
int warning_times_threshold ;
cv::Mat lcw_visual; 
TrackArray fusion_data ;
ld_Frame lane_data ;
};

ICV_REGISTER_FUNCTION(LCWFunction)

#endif  //
