
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
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>   
#include <opencv2/highgui/highgui.hpp>   
#include <opencv2/imgproc/imgproc.hpp>   
#include <opencv2/features2d/features2d.hpp>  
#include <string>
#include <sstream>
#include <vector>
using namespace icv;
using namespace core;
using namespace std;

class SimpleCamFunction : public icvFunction
{
public:
    SimpleCamFunction(icv_shared_ptr<const icvMetaData> info) : icvFunction(info) {
	
		
		if (!mWebcam.open(0))
			throw std::runtime_error("cannot open the requested device");
		
		
		/*if (!mWebcam.set(CV_CAP_PROP_FRAME_WIDTH, 800))
			throw std::runtime_error("cannot set the requested width");
		if (!mWebcam.set(CV_CAP_PROP_FRAME_HEIGHT, 600))
			throw std::runtime_error("cannot set the requested height");*/
		//tempdata=new  icv::data::icvCvMatData(600,800,CV_8UC3 );
		tempdata=new  icv::data::icvCvMatData();
	    //Register_Pub_Remote("tcp://127.0.0.1:5555");
		Register_Pub("first_cam");
		//cam_img=new icv::data::Image();

                ICV_LOG_INFO << "Camera Started";

	}

    SimpleCamFunction() : SimpleCamFunction(nullptr) {	}

    virtual void Execute() override
    {
		if (mWebcam.isOpened())
		{
			mWebcam >> mFrame;
			if(mFrame.data) 
			{	
			cv::Mat img_decode;
		    count++;
            stringstream ss;
			tempdata->setvalue(mFrame);
			// tempdata->Serialize(ss,0);
			// cout<<"size of middle product: "<<ss.str().size()<<endl;
			// tempdata->Deserialize(ss,0);

			// std::vector<uchar> data_encode;
            // cv::imencode(".jpg", mFrame, data_encode);
            // std::string str_encode(data_encode.begin(), data_encode.end());
		    // ss<<str_encode;


			// path p("/home/yining/OpenICV/ICVOS/bin/test_picture/imgencode_cplus.txt");
			// 	newfs::ofstream ofs(p);
			// 	assert(ofs.is_open());
			// 	ofs << str_encode;
			// 	//cout<<"ofs size: "<<ofs.str().size()
			// 	ofs.flush();
			// 	ofs.close();
			
			// 	//read image encode file and display
			// 	newfs::fstream ifs(p);
			// 	assert(ifs.is_open());
			// 	std::stringstream sstr;
			// 	while(ifs >> sstr.rdbuf());
			// 	ifs.close();



			// cout<<"size of one image: "<<ss.str().size()<<endl;
            // std::string str_tmp = ss.str();
			// std::vector<uchar> data(str_tmp.begin(), str_tmp.end());
			// cout<<"size of received data: "<<data.size()<<endl;
            // //img_decode=mFrame;




			// img_decode = cv::imdecode(data, CV_LOAD_IMAGE_COLOR);
			// cout<<"size of received image: "<<img_decode.cols<<" and "<<img_decode.rows<<endl;
            // string savefile = "/home/yining/OpenICV/ICVOS/bin/test_picture/" + to_string(count) + ".jpg"; 
			// imwrite(savefile, img_decode);
            cv::imshow("jk1",tempdata->getvalue()); cv::waitKey(10); 
			icvPublish("first_cam",tempdata);
			
			}
			if (mFrame.data) 
			{
				
			}
		}
    }
private:
int count=0;
std::vector<double> _coeffs;
bool _created=false;
cv::VideoCapture mWebcam;
cv::Mat mFrame, frame_send;
icv::data::icvCvMatData *tempdata; 
icvPublisher* cam_pub;

//icv::data::icvPythonData *cam_data; 

};

ICV_REGISTER_FUNCTION(SimpleCamFunction)


#endif  //
