
#ifndef _DisplayFunction_H
#define _DisplayFunction_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"
#include "OpenICV/Basis/icvSemaphore.h"

//#include "OpenICV/Data/icvCvmatData.hxx"
#include "OpenICV/structure/icvCvMatData.hxx"

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
using namespace icv;
using namespace core;
using namespace std;
using namespace cv;

class DisplayFunction : public icvFunction
{
public:
    DisplayFunction(icv_shared_ptr<const icvMetaData> info) : icvFunction(info) {
		timesync_ = new icvSemaphore(1);
		icv_thread loop(bind(&DisplayFunction::InnerLoop, this));
		img_all.create(784,652,CV_8UC3);
		img_all.setTo(0);
		roi_ssd = cv::Rect(4,4,512,256);
		roi_lane = cv::Rect(4,264,512,256);
		roi_fcw = cv::Rect(520,4,128,256);
		roi_ldw = cv::Rect(520,264, 128,256);
		roi_fusion = cv::Rect(4,524,512,256);
		start_ = icvTime::now_us().time_since_epoch().count();
		count_ = 0 ;
        //Register_Sub("ssd_image");
		Register_Sub("lanedetection_result");
        Register_Sub("fcw_image");
		Register_Sub("ldw_visual");
		//Register_Sub("fusion_image");
	}

    DisplayFunction() : DisplayFunction(nullptr) {	}

    virtual void Execute() override
    {
		// ICV_LOG_INFO<<"display ";
        timesync_->Lock();     
		icv::data::icvCvMatData img_ssd, img_lane, img_fcw, img_ldw, img_fusion;
		//icvSubscribe("ssd_image",&img_ssd);
		icvSubscribe("lanedetection_result",&img_lane);
        icvSubscribe("fcw_image",&img_fcw);
		icvSubscribe("ldw_visual",&img_ldw);
		if(img_lane.is_not_empty()){
		img_all(roi_lane)=img_lane.getvalue();
		imshow("Display",img_lane.getvalue());
		waitKey(10);}
		
		
		if(img_fcw.is_not_empty()){
		img_all(roi_fcw)=img_fcw.getvalue();
		imshow("FCW",img_fcw.getvalue());
		waitKey(10);}
		
		if(img_ldw.is_not_empty()){
		img_all(roi_fcw)=img_ldw.getvalue();
		imshow("LDW",img_ldw.getvalue());
		waitKey(10);}
		
		
		//icvSubscribe("fusion_image",&img_fusion);
        /*
        if (img_lane.is_not_empty()&&img_fcw.is_not_empty()&&img_ldw.is_not_empty())
        {
			//img_all(roi_ssd)=img_ssd.getvalue();
			//img_ssd.CopyTo(img_all(roi_ssd));
			img_all(roi_lane)=img_lane.getvalue();
            img_all(roi_fcw)=img_fcw.getvalue();
			//img_lane.CopyTo(img_all(roi_lane));
			//img_fcw.CopyTo(img_all(roi_fcw));
			img_all(roi_ldw)=img_ldw.getvalue();
			//img_ldw.CopyTo(img_all(roi_ldw));
			//img_all(roi_fusion)=img_fusion.getvalue();
			//img_fusion.CopyTo(img_all(roi_fusion));
			end_ = icvTime::now_us().time_since_epoch().count();
			count_ ++ ;
			ICV_LOG_INFO<<"\n========= Mean Display time: "<<(end_ - start_)/1000/count_ <<" ms";
			imshow("Display_all",img_all);
			waitKey(10);
		}
		*/
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
cv::Mat img_ssd, img_lane, img_fcw, img_ldw, img_fusion,img_all ;
cv::Rect roi_ssd, roi_lane, roi_fcw, roi_ldw, roi_fusion;
time_t start_, end_ ;
int count_ ;
};

ICV_REGISTER_FUNCTION(DisplayFunction)


#endif  //
