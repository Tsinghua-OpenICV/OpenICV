
#ifndef _ForwardCollisionWarning_H
#define _ForwardCollisionWarning_H

#include <boost/thread/thread.hpp>
#include <iostream>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>   
#include <opencv2/highgui/highgui.hpp>   
#include <opencv2/imgproc/imgproc.hpp>   
#include <opencv2/features2d/features2d.hpp>  
#include <mutex>
#include <string>
#include <sstream>
#include <stdio.h>

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Basis/icvStructureData.hxx"
#include "OpenICV/Basis/icvSemaphore.h"

#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"
#include "OpenICV/structure/icvCvMatData.hxx"

#include "OpenICV/structure/structureLaneData.h"
#include "OpenICV/structure/structureIMU.h"
#include "OpenICV/structure/structureFusionData.h"



using namespace icv;
using namespace core;
using namespace std;
using namespace cv ;

typedef data::icvStructureData<ld_Frame> icvlane ;
typedef data::icvStructureData<TrackArray> icvfusion ;

class ForwardCollisionWarning : public icvFunction
{
public:
    ForwardCollisionWarning(icv_shared_ptr<const icvMetaData> info) : icvFunction(info) {
		vx_SV=0.0;
		vy_SV=0.0;
		ax_SV=0.0;
		ay_SV=0.0;
		flag_curve=0;
		lane_width=3.5;
		lane_radius=1000;
		last_level=0;                
		flag_warning=0;               
		working_condition=0;         
		warning_num;  
		timesync_=new icvSemaphore(1);	
		//icv_thread loop(bind(&ForwardCollisionWarning::InnerLoop, this));
		// ICV_LOG_INFO<<"##########################";
		img_data = new  icv::data::icvCvMatData();	
		// ICV_LOG_INFO<<"@@@@@@@@@@@@@@@@@@@@@@@@@@";
		icv_thread loop(bind(&ForwardCollisionWarning::InnerLoop, this));
		
	}

	
	struct Object
	{
		int id=-1;
		float disx=0.0;
		float disy=0.0;
		float dis=1000.0;
		float vx=0.0;
		float vy=0.0;
		float ax=0.0;
		float ay=0.0;
		float dis_warning = 1000.0;
		float dis_critical = 1000.0;
	};

	void callback_laneinfo(const ld_Frame &msg)
	{
		lane_width=msg.lane_width;
		lane_radius=msg.curve_radius;
		if ( lane_width<2.5 || lane_width>4.5)
			lane_width = 3.5;
		if (lane_radius>=1000)
			flag_curve=0;
		else if((lane_radius>=500)&&(lane_radius<1000))
			flag_curve=1;
		else if ((lane_radius>=250)&&(lane_radius<500))
			flag_curve=2;
		else if ((lane_radius>=125)&&(lane_radius<250))
			flag_curve=3;
		else
			flag_curve=4;
	}



	void callback_fusion(const TrackArray &msg) 
	{ 
		ROI_info.clear();
		vx_SV=msg.self_velocity.x;
		vy_SV=msg.self_velocity.y;
		ax_SV=msg.self_acceleration.x;
		ay_SV=msg.self_acceleration.y;	
		
		int i=0;
		//Set parameters according to GB∕T 33577-2017
		float d0=2.0;// Min_Detection distance without detection ability(m)
		float tmin=0.4;//Min_React time of drivers(s)
		float tmax=1.5;//Max_React time of drivers(s)
		float vmin=5;//Min_velocity of self_car(m/s)
		float vmax=50;//Min_velocity of self_car(m/s)
		float amin=3.6;//Min_deceleration of self_car(m/s^2)
		float v_relmax=30;//Max_relative velocity (m/s)
		float d1=tmin*vmin;// Min_Detection distance without detection ability (m)
		float d2;// Min_Detection distance for objects cut in (m)
		float dmax=v_relmax*tmax+v_relmax*v_relmax/(2*amin);// Max_Detection distance(m)  

		//Define the Range of RoI 
		//Define d2 according to lane_radius
		if(flag_curve<=1)
			d2=10;
		else if(flag_curve==2)
			d2=7.5;
		else if(flag_curve==3)
			d2=5;
		float veh_leng=4.0;
		float veh_width=2.5;
		float theta = 0;
		float theta1 = 0;
		float r2;
		for (int j=0;j<msg.tracks.size();j++)
		{
			if ((msg.tracks[j].obs_position.x-3.5)>0)   //Frontal object  minus 3.5 because ros_fusion result has 3.5m offset in longitudinal direction
			{

				if((d0<(msg.tracks[j].obs_position.x-3.5))&&((msg.tracks[j].obs_position.x-3.5-veh_leng/2)<dmax)) //longitudinal range
				{
					if((-lane_width/2+1<(msg.tracks[j].obs_position.y))&&((msg.tracks[j].obs_position.y)<lane_width/2-1)) //Considering boundary condition
					{
						ROIobject.id = msg.tracks[j].id;
						if (flag_curve==0||flag_curve==1){//straightaway
							ROIobject.disx = msg.tracks[j].obs_position.x-3.5;
							ROIobject.disy = msg.tracks[j].obs_position.y;
							ROIobject.dis =  ROIobject.disx;   //Using longitudinal distance to compare with safety distance
							ROIobject.vx = msg.tracks[j].velocity.x;
							ROIobject.vy = msg.tracks[j].velocity.y;
							ROIobject.ax = msg.tracks[j].acceleration.x;
							ROIobject.ay = msg.tracks[j].acceleration.y;
							ROI_info.push_back(ROIobject);  //Insert new object image
							}
						else if (flag_curve==2||flag_curve==3) {  // curve 
							
							ROIobject.disx = msg.tracks[j].obs_position.x-3.5;
							ROIobject.disy = msg.tracks[j].obs_position.y;
							theta=2*asin(sqrt(pow(ROIobject.disx,2)+pow(ROIobject.disy,2))/2/lane_radius);//calculate steering angle;
							ROIobject.dis = lane_radius*theta;  //Using path length to compare with safety distance
							ROIobject.vx = msg.tracks[j].velocity.x*cos(theta)+msg.tracks[j].velocity.y*sin(theta);
							ROIobject.vy = -msg.tracks[j].velocity.x*sin(theta)+msg.tracks[j].velocity.y*cos(theta);
							ROIobject.ax = msg.tracks[j].acceleration.x*cos(theta)+msg.tracks[j].acceleration.y*sin(theta);
							ROIobject.ay = -msg.tracks[j].acceleration.x*sin(theta)+msg.tracks[j].acceleration.y*cos(theta);
							ROI_info.push_back(ROIobject);
							} 
					}
				}
			}	
		}
	}


	void WarningGui(int flag_warning,const Object Warning_object){
		cv::Mat fcw_visual;
		fcw_visual.create(256, 128, CV_8UC3);
		fcw_visual.setTo(255);
		cv::line(fcw_visual, cv::Point2f(4,30), cv::Point2f(4, 246), cv::Scalar(0, 255, 0), 4, 8, 0); 
		cv::line(fcw_visual, cv::Point2f(124, 30), cv::Point2f(124, 246), cv::Scalar(0,255,0), 4, 8, 0);

		float x;
		float y;
		string str ;

		int roi_y = 30;

		if ((0<=vx_SV )&&(vx_SV<=50)&&(flag_curve<4)) //Start conditions
		{
			if (flag_warning!=0)
			{
				str = "danger";
				cv::putText(fcw_visual, str, cv::Point2f(30,20),cv::FONT_HERSHEY_DUPLEX,0.5,CV_RGB(255,0,0),0.2,8,0);

				if (Warning_object.disx > 10)
				{
					roi_y = 30 ;
				}else
				{
					roi_y = 150 - Warning_object.disx*12 ;
				}			
				cv::circle(fcw_visual, cv::Point2f(64,roi_y),5,cv::Scalar(255,0,0),2,8,0);
			}
			else
			{
				str = "safe ";
				cv::putText(fcw_visual, str, cv::Point2f(35,20),cv::FONT_HERSHEY_DUPLEX,0.5,CV_RGB(0,255,0),0.2,8,0);
			}
		}else
		{
			str = "safe ";
			cv::putText(fcw_visual, str, cv::Point2f(35,20),cv::FONT_HERSHEY_DUPLEX,0.5,CV_RGB(0,255,0),0.2,8,0);
		}

		//cv::imshow("FCW",fcw_visual);
		//cv::waitKey(10);
		img_data->setoutvalue(fcw_visual);
		Send_Out(img_data,0);
	}

	void risk(float vx_SV,float vy_SV,float ax_SV,float ay_SV,const vector<Object>& ROI_info)
	{
		if (flag_warning!=0)
			last_level = 1;
		else
			last_level = 0;
		flag_warning = 0;
		risk_info.clear();
		float T_react = 1.2;
		float d0=3.0;
		float a1=3.6;
		float a2=7;
		int num=ROI_info.size();
		float dis_warning[num];
		float dis_critical[num];
		int flag_risk=0;
		int flag_warning_condition=0;
		for (int i=0;i<ROI_info.size();i++){
			float vx_FV = vx_SV + ROI_info[i].vx;
			float vy_FV = vy_SV + ROI_info[i].vy;
			float ax_FV = ax_SV + ROI_info[i].ax;
			float ay_FV = ay_SV + ROI_info[i].ay;
			float a_rel = ax_SV-ax_FV;
			if((abs(ax_FV)<0.5) &&(abs(ax_SV)<0.5))
			{
				working_condition =1;
				dis_warning[i]=vx_SV*T_react+(vx_SV*vx_SV-vx_FV*vx_FV)/(2*a1)+d0;
				dis_critical[i]= -ROI_info[i].vx*T_react+0.5*a2*pow(T_react,2);   
				flag_warning_condition=4;
			} 
			else if(abs(a_rel)<0.7)                  
			{
				if(abs(ax_SV)<3.6){
					working_condition=2;
					dis_warning[i]=vx_SV*(T_react+ROI_info[i].vx/a1)-pow(ROI_info[i].vx,2)/(2*a1)+d0;
					dis_critical[i]= -ROI_info[i].vx*T_react;
					flag_warning_condition=3;
			}
			else{
					working_condition=2;
					dis_warning[i]=vx_SV*(T_react+ROI_info[i].vx/ax_SV)-pow(ROI_info[i].vx,2)/(2*ax_SV)+d0;
					dis_critical[i]= -ROI_info[i].vx*T_react;
					flag_warning_condition=3;
			}
				
			}
			else if((vx_FV<1))  
			{
				working_condition =3;
				dis_warning[i]=vx_SV*T_react+vx_SV*vx_SV/(2*a1)+d0;
				dis_critical[i]= vx_SV*T_react+0.5*a2*pow(T_react,2);
				flag_warning_condition=2;
			}
			else if((abs(ROI_info[i].vx)<0.5)&&(ax_FV<0))   
			{
				working_condition = 4;
				dis_warning[i]=vx_SV*T_react+d0;
				dis_critical[i]= 0.5*a2*pow(T_react,2);
				flag_warning_condition=1;
			} 
			else                 
			{
				working_condition = 5;
				dis_warning[i] = vx_SV*T_react+(vx_SV*vx_SV-vx_FV*vx_FV)/(2*a_rel)+d0;
				dis_critical[i] = -ROI_info[i].vx*T_react+0.5*a2*pow(T_react,2);
				flag_warning_condition=5;
			}

			// Judge dangerous object				
			if(ROI_info[i].dis<=dis_warning[i])
			{
				Riskobject.id = ROI_info[i].id;
				Riskobject.dis = ROI_info[i].dis;
				Riskobject.disx = ROI_info[i].disx;
				Riskobject.disy = ROI_info[i].disy;
				Riskobject.vx = ROI_info[i].vx;
				Riskobject.vy = ROI_info[i].vy;
				Riskobject.ax = ROI_info[i].ax;
				Riskobject.ay = ROI_info[i].ay;
				Riskobject.dis_warning = dis_warning[i];
				Riskobject.dis_critical = dis_critical[i];
				risk_info.push_back(Riskobject);
				flag_risk=1;
			}
		}
		//Choose dangerous object
		float min_diswarning=200.0;
		int warning_id=0;
		if(flag_risk == 1){                               
			for(int i=0;i<risk_info.size();i++)
			{
				if(risk_info[i].dis_warning<=min_diswarning)
				{
					min_diswarning=risk_info[i].dis_warning;
					warning_id = i;	
				}
			}
			Warning_object.id = risk_info[warning_id].id;
			Warning_object.dis_critical = risk_info[warning_id].dis_critical;
			Warning_object.dis_warning = risk_info[warning_id].dis_warning;
			Warning_object.dis = risk_info[warning_id].dis;
			Warning_object.disx = risk_info[warning_id].disx;
			Warning_object.disy = risk_info[warning_id].disy;
			Warning_object.vx = risk_info[warning_id].vx;
			Warning_object.vy = risk_info[warning_id].vy;
			Warning_object.ax = risk_info[warning_id].ax;
			Warning_object.ay = risk_info[warning_id].ay;	

			save_object.id=Warning_object.id;
			save_object.dis_critical = Warning_object.dis_critical ;
			save_object.dis_warning = Warning_object.dis_warning;
			save_object.disx = Warning_object.disx;
			save_object.disy = Warning_object.disy;
			save_object.vx = Warning_object.vx;
			save_object.vy = Warning_object.vy;
			save_object.ax = Warning_object.ax;
			save_object.ay = Warning_object.ay;
			if (Warning_object.dis <= Warning_object.dis_critical)
				flag_warning=2;
			else
				flag_warning=1;
		}	
			

		//连续预警问题
		if((last_level==1) && (flag_warning==0)){  
			//出现了上一帧预警而当前帧不预警的情况，把最后一帧预警目标再连续预警20帧
			warning_num++;
			if(warning_num>20){
				warning_num=0;
				last_level = 0;
			}		
			else{
				// 把上一帧的warning结果进行预警
				flag_warning = 3;
				last_level = 1;
				Warning_object.id = save_object.id;
				Warning_object.disx = save_object.disx + save_object.vx * 0.1 * num;
				Warning_object.disy = save_object.disy + save_object.vy * 0.1 * num;
				Warning_object.vx = save_object.vx;
				Warning_object.vy = save_object.vy;
				Warning_object.ax = save_object.ax;
				Warning_object.ay = save_object.ay;
			}
		}
		
		//预警log显示
		// if (flag_warning!=0){
		// 	ICV_LOG_INFO<<"Dangerous!!!";
		// }
		// else{
		// 	ICV_LOG_INFO<<"The situation is safe!";
		// }
	}

    virtual void Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts) override
    {
		timesync_->Lock();
		lanedata = read_Input<icvlane>(0);
		fusiondata = read_Input<icvfusion>(1);
		if (is_not_empty(0)&&is_not_empty(1))
		{
			callback_laneinfo(lanedata);
			callback_fusion(fusiondata);
			risk( vx_SV, vy_SV, ax_SV, ay_SV, ROI_info);
			WarningGui(flag_warning,Warning_object);
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
icvSemaphore* timesync_;

ld_Frame lanedata ;
TrackArray fusiondata ;

float vx_SV=0.0;
float vy_SV=0.0;
float ax_SV=0.0;
float ay_SV=0.0;
int flag_curve=0;
float lane_width=3.5;
float lane_radius=1000;
Object ROIobject;
vector<Object>ROI_info;
Object Riskobject;
vector<Object>risk_info;
Object Warning_object;
Object save_object;
int last_level=0;                
int flag_warning=0;               
int working_condition=0;         
int warning_num;  
icv::data::icvCvMatData *img_data;               
};

ICV_REGISTER_FUNCTION(ForwardCollisionWarning)


#endif  //
