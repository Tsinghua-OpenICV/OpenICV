
#ifndef _ReadvideoFunction_H
#define _ReadvideoFunction_H

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
using namespace icv;
using namespace core;
using namespace std;

class ReadvideoFunction : public icvFunction
{
public:
    ReadvideoFunction(icv_shared_ptr<const icvMetaData> info) : icvFunction(info) 
		{
			ICV_LOG_TRACE << "started";
			if (!video.open(videoFile)) 
			{
				cout<<"Fail to open specified video file:" << videoFile << endl;
				exit(-1);
			}
			tempdata=new  icv::data::icvCvMatData(256,512,CV_8UC3 );tempdata->Reserve();
	  }
    ReadvideoFunction() : ReadvideoFunction(nullptr) {}

    virtual void Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts) override
    {
			int totalFrames = video.get(CV_CAP_PROP_FRAME_COUNT);
			int currentFrames = 0 ;

			while (1)
			{
				video >> mFrame ;
				tempdata->setoutvalue(mFrame);
				Send_Out(tempdata,0);
				if (currentFrames == totalFrames -1)
				{
					currentFrames = 0 ;
					video.set(CV_CAP_PROP_POS_FRAMES, 0) ;
				}
				cv::imshow("jk1",mFrame);  
				cv::waitKey(10);
				currentFrames ++ ;
			}
    }
private:
std::vector<double> _coeffs;
cv::VideoCapture video;
string videoFile = "/home/linaro/dnndk-ssd-latest/dnndk-ssd/adas.avi";
bool _created=false;
cv::VideoCapture mWebcam;
cv::Mat mFrame, frame_send;
icv::data::icvCvMatData *tempdata; 
icv_mutex dataoutlock;
};

ICV_REGISTER_FUNCTION(ReadvideoFunction)


#endif  //
