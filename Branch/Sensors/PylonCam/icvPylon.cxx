
#ifndef _PylonCam_H
#define _PylonCam_H

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
#include <cassert>
#include <stdexcept>
#include <mutex>

#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>   
#include <opencv2/highgui/highgui.hpp>   
#include <opencv2/imgproc/imgproc.hpp>   
#include <opencv2/features2d/features2d.hpp>  

#include "pylon/PylonIncludes.h"
#include <pylon/PylonImage.h>
#include <pylon/Pixel.h>
// #include <pylon/PylonGUI.h>
#include <pylon/gige/_BaslerGigECameraParams.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon/ImageFormatConverter.h>



using namespace icv;
using namespace core;
using namespace std;
using namespace Pylon;
using namespace cv;
using namespace Basler_GigECamera;

class PylonCam : public icvFunction
{
public:
    PylonCam(icv_shared_ptr<const icvMetaData> info) : icvFunction(info) {
	
		ICV_LOG_TRACE << "started";
		open();
		//tempdata=new  icv::data::icvCvMatData(1200,1920,CV_8UC3 );
		Register_Pub("PylonCam_image");		
	}

    PylonCam() : PylonCam(nullptr) {	}

    virtual void Execute() override
    {
	
		retrieveFramegige();
		usleep(100);
		tempdata.setvalue(mFrame);
		icvPublish("PylonCam_image",&tempdata)
		
		// if (!_created)
		// {
		// 	cv::namedWindow("jk1");cv::startWindowThread();
		// 	_created = true;
		// }

		// if (mFrame.data) 
		// {
		// 	cv::imshow("jk1",mFrame); 
		// 	cv::waitKey(20); 
		// }

    }
	void open()
	{
		PylonInitialize();
		DeviceInfoList listdevice;
		TlInfoList listTl;
		if (CTlFactory::GetInstance().EnumerateDevices(listdevice)>0)
			printf("have device", CTlFactory::GetInstance().EnumerateDevices(listdevice));
		else printf("no device");
		CTlFactory::GetInstance().EnumerateTls( listTl );
		int numofcam=listdevice.size();
		if(numofcam>0)
		{
			camera1.Attach(CTlFactory::GetInstance().CreateDevice(listdevice.front()));
			camera1.MaxNumBuffer = 5;
			uint32_t c_countOfImagesToGrab = 100000;
			camera1.Open();	
			camera1.Width.SetValue(1920);
			camera1.Height.SetValue(1200);
			camera1.BalanceWhiteAuto.SetValue(BalanceWhiteAuto_Continuous);
			camera1.ExposureTimeAbs.SetValue(13000.0);
			camera1.StartGrabbing( c_countOfImagesToGrab,GrabStrategy_LatestImageOnly);	
			camera1.AcquisitionFrameRateEnable.SetValue(true);
			camera1.AcquisitionFrameRateAbs.SetValue(10.0);
		}
	}

	void retrieveFramegige()
	{
		camera1.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);
		EPixelType pixelType = ptrGrabResult->GetPixelType();
		uint32_t width = ptrGrabResult->GetWidth();
		uint32_t height = ptrGrabResult->GetHeight();
		mbalser=Mat(height,width,CV_8U,ptrGrabResult->GetBuffer());
		cvtColor(mbalser,mFrame_gige, COLOR_BayerRG2RGB );
		resize(mFrame_gige, mFrame, Size(width/3, height/3), 0, 0, 1); 
		ptrGrabResult.Release();       
	}
private:
bool _created=false;
int count_=0;
cv::Mat mFrame, mbalser,mFrame_gige;
icv::data::icvCvMatData tempdata(1200,1920,CV_8UC3 );
CBaslerGigEInstantCamera camera1 ;
CGrabResultPtr ptrGrabResult;
};

ICV_REGISTER_FUNCTION(PylonCam)


#endif  //
