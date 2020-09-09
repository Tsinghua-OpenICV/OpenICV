
#ifndef _SimpleCamFunction_H
#define _SimpleCamFunction_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"

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

class SimpleCamFunction : public icvFunction
{
public:
    SimpleCamFunction(icv_shared_ptr<const icvMetaData> info) : icvFunction(info) {
	
		ICV_LOG_TRACE << "started";
		if (!mWebcam.open(0))
			throw std::runtime_error("cannot open the requested device");
		/*if (!mWebcam.set(CV_CAP_PROP_FRAME_WIDTH, 800))
			throw std::runtime_error("cannot set the requested width");
		if (!mWebcam.set(CV_CAP_PROP_FRAME_HEIGHT, 600))
			throw std::runtime_error("cannot set the requested height");*/
		//tempdata=new  icv::data::icvCvMatData(600,800,CV_8UC3 );
		tempdata=new  icv::data::icvCvMatData();
		 cam_pub=Register_Pub("first_cam");

	}

    SimpleCamFunction() : SimpleCamFunction(nullptr) {	}

    virtual void Execute() override
    {
		if (mWebcam.isOpened())
		{
			mWebcam >> mFrame;
			if(mFrame.data) 
			{	
				
			tempdata->setvalue(mFrame);
			cout<<"Original image name is "<<tempdata->getImageName()<<endl;
			icvPublish("first_cam",tempdata);
			//cam_pub->set_send_data(tempdata);
			//cam_pub->Send_Out();
			count_++ ;
			}

	 		if (!_created)
            {
               cv::namedWindow("jk1");cv::startWindowThread();
                _created = true;
            }

			if (mFrame.data) 
			{
				cv::imshow("jk1",mFrame); //cv::waitKey(20); 
			}
		}
    }
private:
std::mutex mlock;
std::vector<double> _coeffs;
bool _created=false;
cv::VideoCapture mWebcam;
int count_=0;
cv::Mat mFrame, frame_send;
icv::data::icvCvMatData *tempdata; 
icv_mutex dataoutlock;
icvPublisher* cam_pub;
};

ICV_REGISTER_FUNCTION(SimpleCamFunction)


#endif  //
