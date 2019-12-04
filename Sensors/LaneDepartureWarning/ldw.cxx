
#ifndef _LaneDepartureWarning_H
#define _LaneDepartureWarning_H

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
#include <stdlib.h>
#include <time.h>

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

using namespace icv;
using namespace core;
using namespace std;
using namespace cv ;

#define THRES_LDW  0.1
#define THRES_LDW2 0.25
typedef data::icvStructureData<ld_Frame> icvlane ;
typedef data::icvStructureData<TwistWithCovarianceStamped>    icvvel;


class LaneDepartureWarning : public icvFunction
{
public:
    LaneDepartureWarning(icv_shared_ptr<const icvMetaData> info) : icvFunction(info) 
	{
		hist_v_LaneWidth.clear();
    	vector_InitValue(hist_v_LaneWidth, 3.5, 10);
		average_speed=-1;
		dep_speed=-1;
		speed = -1 ;
		hist_lanewidth=4.8;
		flag_speed=1;
		mid = 0; 
		bias_dis = 0;
		lane_width = 4.8;
		timesync_ = new icvSemaphore(1);
		icv_thread loop(bind(&LaneDepartureWarning::InnerLoop, this));
		img_data = new icv::data::icvCvMatData(128,258, CV_8UC3);
		
	}
	double valueAtIPM(std::vector<float> &f, float x)
	{
		float ans = 0.f;
		for (int i = (int)f.size() - 1; i >= 0; --i)
			ans = ans * x + f[i];
		return ans;
	}

	void vector_InitValue(std::vector<float> & vector_a,float value,int num)
	{
		if(vector_a.size()!=0){
			vector_a.clear();
		}else{
			for(int ii=0;ii<num;ii++){
			vector_a.push_back(value);
			}
		}
	}

	void vector_Update(std::vector<float> &vector_a, float value)
	{
		vector_a.push_back(value);
		vector_a.erase(vector_a.begin());
	}

	void vel_callback(const TwistWithCovarianceStamped &msg)
	{
		double vx = msg.twist.twist.linear.x;
		double vy = msg.twist.twist.linear.y;
		double vz = msg.twist.twist.linear.z;
		speed=sqrt(vx*vx+vy*vy+vz*vz);
	}

	void visual()
	{
		cv::Mat ldw_visual; 
		ldw_visual.create(256, 128, CV_8UC3);
		ldw_visual.setTo(255);
		
		if (right_dis < THRES_LDW2 && dep_speed > 0)
		{
			if(right_dis < THRES_LDW || dep_speed > 0.4){
				
				cv::line(ldw_visual, cv::Point2f(124, 30), cv::Point2f(124, 246), cv::Scalar(0, 0, 255), 4, 8, 0); //红色为报警
				cv::line(ldw_visual, cv::Point2f(4, 30), cv::Point2f(4, 246), cv::Scalar(0, 255, 0), 2, 4, 0);
				str_tmp = "right ";
				cv::putText(ldw_visual, str_tmp, cv::Point2f(35,20),cv::FONT_HERSHEY_DUPLEX,0.5,CV_RGB(255,0,0),0.2,8,0);
			}
		}
		else if (left_dis < THRES_LDW2 && dep_speed < 0)
		{
			if (left_dis < THRES_LDW || dep_speed < -0.4)
			{
				
				cv::line(ldw_visual, cv::Point2f(4, 30), cv::Point2f(4, 246), cv::Scalar(0, 0, 255), 4, 8, 0);
				cv::line(ldw_visual, cv::Point2f(124, 30), cv::Point2f(124, 246), cv::Scalar(0, 255, 0), 4, 8, 0);
				str_tmp = "left ";//situation is safe!" ;
				cv::putText(ldw_visual, str_tmp, cv::Point2f(35,20),cv::FONT_HERSHEY_DUPLEX,0.5,CV_RGB(255,0,0),0.2,8,0);	
			}
		}
		else
		{
			cv::line(ldw_visual, cv::Point2f(4, 30), cv::Point2f(4, 246), cv::Scalar(0, 255, 0), 4, 8, 0);
			cv::line(ldw_visual, cv::Point2f(124, 30), cv::Point2f(124, 246), cv::Scalar(0, 255, 0), 4, 8, 0);
			str_tmp = "safe ";
			cv::putText(ldw_visual, str_tmp, cv::Point2f(35,20),cv::FONT_HERSHEY_DUPLEX,0.5,CV_RGB(0,255,0),0.2,8,0);			
		}
		double car_x = 64 + 55 * bias_dis / (lane_width) ;
		
		cv::circle(ldw_visual, cv::Point2f(car_x, 150), 7, cv::Scalar(0, 255, 20), 5, 8, 0);
		// cv::imshow("LDW", ldw_visual);
		// cv::waitKey(10);
		img_data->setoutvalue(ldw_visual);
		Send_Out(img_data,0);
	}

	void ldw_callback(const ld_Frame &msg)
	{
		for (int ii = 0; ii < 10; ii++){
			hist_lanewidth += hist_v_LaneWidth[hist_v_LaneWidth.size() - 1 - ii];
		}
		hist_lanewidth = hist_lanewidth / 11.0;
		std::vector<double> candidate_dis;
		std::vector<int> candidate_id;
		std::vector<float> Lane_angle(2);

		candidate_id.clear();
		candidate_dis.clear();
		Lane_angle.clear();

		//read lane data
		for(int i=0;i<msg.lane_Coeff.size();i++)
		{
			cv::Point2f dot_p;
			std::vector<float> coeff(4);
			coeff[3] = msg.lane_Coeff[i].a;
			coeff[2] = msg.lane_Coeff[i].b;
			coeff[1] = msg.lane_Coeff[i].c;
			coeff[0] = msg.lane_Coeff[i].d;

			dot_p.x = 0;
			dot_p.y = valueAtIPM(coeff, 0);

			if (-hist_lanewidth-0.5 < dot_p.y && dot_p.y < hist_lanewidth+0.5)
			{
				lane_angle = atan(3 * coeff[3] * dot_p.x * dot_p.x + 2 * coeff[2] * dot_p.x + coeff[1]);
				candidate_dis.push_back(dot_p.y*cos(lane_angle));
				candidate_id.push_back(i);
				Lane_angle.push_back(lane_angle);
			}
		}

		double right_0 = 0.0;
		double left_0 = 0.0;
		double right_angle = 0.0;
		double left_angle = 0.0;
		int flag_right = 0;
		int flag_left = 0;

		bool b_flag1 = true; 
		bool b_flag2 = true; 

		//finding the nearest right lane and the nearest left lane
		if (candidate_dis.size() > 0){
			for (int ii = 0; ii < candidate_dis.size(); ii++){
				if (candidate_dis[ii] < 0){
					if (b_flag1){
						right_0 = candidate_dis[ii];
						right_angle = Lane_angle[ii];
					}
					b_flag1 = false;

					if (candidate_dis[ii] > right_0)
					{
						right_0 = candidate_dis[ii];
						right_angle = Lane_angle[ii];
					}
					flag_right = 1;
				}
				if (candidate_dis[ii] > 0){
					if (b_flag2){
						left_0 = candidate_dis[ii];
						left_angle = Lane_angle[ii];
					}
					b_flag2 = false;

					if (candidate_dis[ii] < left_0){
						left_0 = candidate_dis[ii];
						left_angle = Lane_angle[ii];
					}
					flag_left = 1;
				}
			}
		}
		
		//LDW输出
		//正常的双边状态
		if(flag_speed == 1 && speed != -1){
			vector_InitValue(hist_speed, speed, 3);
			flag_speed = 0;
		}
		else if(flag_speed == 0 && speed != -1){
			vector_Update(hist_speed,speed);
		}
		else{
			vector_InitValue(hist_speed, -1, 3);
		}
		average_speed=0;
		for(int i=0;i<3;i++){
			average_speed += hist_speed[i];
		}
		average_speed=average_speed/3.0;
		
		if (flag_right && flag_left && (fabs(right_0) + fabs(left_0)) * cos(left_angle) > 2.8 \
			&& (fabs(right_0) + fabs(left_0)) * cos(left_angle) < 4.8)
		{
			mid = (right_0 + left_0) / 2;
			angle_final = (right_angle + left_angle)/2+0.02;
			bias_dis = mid * cos(angle_final);
			right_dis = (fabs(right_0)-0.761)*cos(angle_final);
			left_dis = (fabs(left_0)-0.761)*cos(angle_final);
			lane_width = (fabs(right_0) + fabs(left_0)) * cos(angle_final);
			dep_speed = average_speed*sin(angle_final);

			//如果道路宽度有突变，则参考历史容器中的大小，做为纠正值
			if (fabs(lane_width - hist_v_LaneWidth[hist_v_LaneWidth.size() - 1]) > 0.3)
			{
				for (int ii = 0; ii < 5; ii++)
				{
					lane_width += hist_v_LaneWidth[hist_v_LaneWidth.size() - 1 - ii];
				}
				lane_width = lane_width / 6.0;
			}     
			//将道路宽度加入历史容器中
			if ((fabs(right_0) + fabs(left_0)) * cos(angle_final) > 2.8){
				vector_Update(hist_v_LaneWidth, lane_width);
			}  
		}
		else
		{
			//现在是不准确的，预测状态
			for (int ii = 0; ii < 10; ii++)
			{
				lane_width += hist_v_LaneWidth[hist_v_LaneWidth.size() - 1 - ii];
			}
			lane_width = lane_width / 11.0;
			vector_Update(hist_v_LaneWidth, lane_width);

			//单边
			if((flag_left && !flag_right) || (!flag_left && flag_right)){

				if(flag_left){
					right_0=left_0-lane_width;
				}
				else{
					left_0=right_0+lane_width;
				}

				mid = (right_0 + left_0) / 2;
				angle_final = left_angle + right_angle+0.02; 
				bias_dis = mid * cos(angle_final);
				right_dis = (fabs(right_0)-0.761) * cos(angle_final);
				left_dis = (fabs(left_0)-0.761) * cos(angle_final);
				dep_speed = average_speed * sin(angle_final);
			}
			else{
				if(!flag_right && !flag_left){
					ICV_LOG_INFO<<"No ego lanes";
				}
				else{
					ICV_LOG_INFO<<"Too narrow lanes";
				}
			}
		}  	
	}
    virtual void Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts) override
    {
		timesync_->Lock();
		// read lane detection output and imu vel data
		lanedata = read_Input<icvlane>(0);
		veldata = read_Input<icvvel>(1);
		if (is_not_empty(0)&&is_not_empty(1))
		{
			// warning decision with ldw model
			vel_callback(veldata);
			ldw_callback(lanedata);

			// visualize ldw warning with opencv 
			visual();	
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
TwistWithCovarianceStamped veldata;

double average_speed;
double dep_speed;
double speed ;
double lane_angle;
double angle_final;
std::vector<float> hist_v_LaneWidth;
double hist_lanewidth;
std::vector<float> hist_speed;
int flag_speed;
string str_tmp;
double mid ; 
double right_dis;
double left_dis;
double bias_dis;
double lane_width;
icv::data::icvCvMatData *img_data;               

};

ICV_REGISTER_FUNCTION(LaneDepartureWarning)


#endif  //
