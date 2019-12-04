
#ifndef _ImageSub_H
#define _ImageSub_H

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


class ImageSub : public icvFunction
{
public:
    ImageSub(icv_shared_ptr<const icvMetaData> info) : icvFunction(info) {
		start1_ = clock();
    count1_ = 0;
    alltime = 0 ;
    meantime = 0 ;
    source_time = 0 ;
	}

  virtual void Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts) override
  {
    mFrame=read_Input<icv::data::icvCvMatData>(0);
    if (mFrame.cols>0)
    {
      imshow("jk1",mFrame);
      waitKey(100);   
    }
    usleep(100000);
  }
private:
  clock_t start1_, end1_ ;
  int count1_ ;
  cv::Mat mFrame;
  time_t source_time, time_now ;
  time_t alltime, meantime ;
};

ICV_REGISTER_FUNCTION(ImageSub)


#endif  //
