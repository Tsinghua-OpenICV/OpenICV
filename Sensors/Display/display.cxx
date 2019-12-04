
#ifndef _DisplayFunction_H
#define _DisplayFunction_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
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
		start_ = SyncClock::now_us().time_since_epoch().count();
		count_ = 0 ;


	}

    DisplayFunction() : DisplayFunction(nullptr) {	}

    virtual void Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts) override
    {
		// ICV_LOG_INFO<<"display ";
        timesync_->Lock();     
        img_ssd = read_Input<icv::data::icvCvMatData>(0);
		img_lane = read_Input<icv::data::icvCvMatData>(1);
		img_fcw = read_Input<icv::data::icvCvMatData>(2);
		img_ldw = read_Input<icv::data::icvCvMatData>(3);
		img_fusion = read_Input<icv::data::icvCvMatData>(4);
        if (is_not_empty(0)&&is_not_empty(1)&&is_not_empty(2)&&is_not_empty(3))
        {
			img_ssd.copyTo(img_all(roi_ssd));
			img_lane.copyTo(img_all(roi_lane));
			img_fcw.copyTo(img_all(roi_fcw));
			img_ldw.copyTo(img_all(roi_ldw));
			img_fusion.copyTo(img_all(roi_fusion));
			end_ = SyncClock::now_us().time_since_epoch().count();
			count_ ++ ;
			ICV_LOG_INFO<<"\n========= Mean Display time: "<<(end_ - start_)/1000/count_ <<" ms";
			imshow("Display",img_all);
			waitKey(10);
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
cv::Mat img_ssd, img_lane, img_fcw, img_ldw, img_fusion,img_all ;
cv::Rect roi_ssd, roi_lane, roi_fcw, roi_ldw, roi_fusion;
time_t start_, end_ ;
int count_ ;
};

ICV_REGISTER_FUNCTION(DisplayFunction)


#endif  //
