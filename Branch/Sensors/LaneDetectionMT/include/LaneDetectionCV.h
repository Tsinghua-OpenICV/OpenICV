   
//////////////////////////////////////////////////////////////////////////////////////////
//
// Author : 
// Date:
// Comment:
// 
//////////////////////////////////////////////////////////////////////////////////////////
#ifndef __LANE_DETECTION_CV_H__
#define __LANE_DETECTION_CV_H__

#include <mutex>
#include <iostream>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <numeric>
#include <time.h>
#include <chrono>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>		 //图像处理
#include <opencv2/highgui/highgui.hpp>		 //opencv GUI

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"

#include "OpenICV/Basis/icvStructureData.hxx"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Basis/icvSemaphore.h"

#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"
#include "OpenICV/structure/icvCvMatData.hxx"
#include "OpenICV/structure/structureIMU.h"
#include "OpenICV/structure/structureFusionData.h"
#include "OpenICV/structure/structureLaneData.h"

#include "paircomp.h"
#include "threadSafePriorQueue.h"
#include "LaneDetection.h"
#include "JudgeLane.h"

using namespace icv;
using namespace core;
using namespace std;
using namespace cv;

typedef data::icvStructureData<ld_Frame> icvlane ;

#define MAX_THREAD_COUNT 5

class LaneDetectionCV : public icvFunction 
{
public:
	explicit LaneDetectionCV(icv_shared_ptr<const icvMetaData> info);
	virtual ~LaneDetectionCV(void);

	// interact callback with ROS reserved
	// void imageCb(const sensor_msgs::ImageConstPtr &msg) ;	
	virtual void Execute() override;

	int process(void);
	void display(void);
	void InnerLoop();

private:
	// ThreadSafePriorQueue<std::pair<int, cv::Mat>,paircomp> input_queue_;
	ThreadSafePriorQueue<InputTaskPair, paircomp_inputtask> input_queue_;
	volatile int input_index_;
	// ThreadSafePriorQueue<std::pair<int, cv::Mat>, paircomp> show_queue_;
	ThreadSafePriorQueue<OutputTaskPair, paircomp_outputtask> show_queue_ ;
	volatile int show_index_;

	std::array<std::shared_ptr<std::thread>, MAX_THREAD_COUNT> threadsList_;

	double t1, t2;
	int frame_count_;
	icvlane *lane_output_;

    icvSemaphore* timesync_;
	icv::data::icvCvMatData *img_data;
};

inline LaneDetectionCV::LaneDetectionCV(
	icv_shared_ptr<const icvMetaData> info) : icvFunction(info), input_queue_(50),  show_queue_(50), 
	input_index_(0), show_index_(0), t1(0.0), t2(0.0), frame_count_(0)
{
    ICV_LOG_TRACE << " Lane DetectionCV started";    
    t1 = clock();
	timesync_=new icvSemaphore(1);
	icv_thread loop(bind(&LaneDetectionCV::InnerLoop, this));
    threadsList_[0] = make_shared<std::thread>(&LaneDetectionCV::display, this);
    for (int i = 1; i < MAX_THREAD_COUNT; i  ++) {
	threadsList_[i] = make_shared<std::thread>(&LaneDetectionCV::process, this);
    }
    lane_output_ = new icvlane();
	img_data = new  icv::data::icvCvMatData();
    ICV_LOG_TRACE << " Lane DetectionCV leave";
	Register_Sub("first_cam");
	Register_Pub("lanedetection_result");
	Register_Pub("lanedata");
}

inline LaneDetectionCV::~LaneDetectionCV(void)
{
    for (int i = 0; i < MAX_THREAD_COUNT; i++) {
       	threadsList_[i]->join();
    }
}

inline void LaneDetectionCV::InnerLoop()
{
	while(true) 
	{
		timesync_->Release();
		usleep(ICV_SLEEP_CYCLE);
	}
}

ICV_REGISTER_FUNCTION(LaneDetectionCV)

#endif
