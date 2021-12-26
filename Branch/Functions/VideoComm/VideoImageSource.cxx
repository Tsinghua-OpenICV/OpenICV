
#ifndef _ImagePub_H
#define _ImagePub_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"
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
#include <ctime>


using namespace icv;
using namespace core;
using namespace std;
using namespace cv;

ICV_CONSTEXPR char KEY_OUTPUT_FILEPATH[] = "path";
class VideoImageSource : public icvFunction
{
public:
    VideoImageSource(icv_shared_ptr<const icvMetaData> info) : icvFunction(info) {
		if (_information.Contains(KEY_OUTPUT_FILEPATH))
        videoFile = _information.GetString(KEY_OUTPUT_FILEPATH);
        else  videoFile="";
		if (!video.open(videoFile)) 
		{
			cout<<"Fail to open specified video file:" << videoFile << endl;
			exit(-1);
		}
		start_ = clock();
		count_ = 0 ;
		i = 0 ;
		//tempdata=new  icv::data::icvCvMatData();
		Register_Pub("video_images");
		// glob(pattern,image_files);
	}

    VideoImageSource() : VideoImageSource(nullptr) {	}

    virtual void Execute() override
    {
		int totalFrames = video.get(CV_CAP_PROP_FRAME_COUNT);

		int currentFrames = 0 ;
		
		video >> mFrame ;
		data::icvCvMatData datatosend(mFrame);
        icvPublish("video_images",&datatosend);//=mFrame;
	    usleep(10000);
	}


private:
cv::VideoCapture video;
string videoFile;
clock_t start_, end_ ;
int count_ ;
int i ;
cv::Mat mFrame;
//icv::data::icvCvMatData *tempdata; 
};

ICV_REGISTER_FUNCTION(VideoImageSource)


#endif  //
