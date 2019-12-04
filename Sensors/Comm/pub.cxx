
#ifndef _ImagePub_H
#define _ImagePub_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
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


class ImagePub : public icvFunction
{
public:
    ImagePub(icv_shared_ptr<const icvMetaData> info) : icvFunction(info) {
		if (!video.open(videoFile)) 
		{
			cout<<"Fail to open specified video file:" << videoFile << endl;
			exit(-1);
		}
		start_ = clock();
		count_ = 0 ;
		i = 0 ;
		tempdata=new  icv::data::icvCvMatData();
		// glob(pattern,image_files);
	}

    ImagePub() : ImagePub(nullptr) {	}

    virtual void Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts) override
    {
		int totalFrames = video.get(CV_CAP_PROP_FRAME_COUNT);
		int currentFrames = 0 ;
		
		video >> mFrame ;
		// imshow("jk2",mFrame);
		// waitKey(100);   
		tempdata->setoutvalue(mFrame);
		Send_Out(tempdata,0);
		usleep(100000);
		if (currentFrames == totalFrames -1)
		{
			currentFrames = 0 ;
			video.set(CV_CAP_PROP_POS_FRAMES, 0) ;
		}
		currentFrames ++ ;

    }
private:
cv::VideoCapture video;
string videoFile = "/home/linaro/dnndk-ssd-latest/dnndk-ssd/adas.avi";
clock_t start_, end_ ;
int count_ ;
int i ;
cv::Mat mFrame;
icv::data::icvCvMatData *tempdata; 
};

ICV_REGISTER_FUNCTION(ImagePub)


#endif  //
