#ifndef __SETINPUTIMAGE_H__
#define __SETINPUTIMAGE_H__
#include "profile.h"

using namespace std;
using namespace cv;

float my_round(float a)
{
		return floor(a + 0.5);
}
#endif

/**
 * @brief Feed input frame into DPU for process
 *
 * @param task - pointer to DPU Task for YOLO-v3 network
 * @param frame - pointer to input frame
 * @param mean - mean value for YOLO-v3 network
 *
 * @return none
 */
void setInputImageForSSD(DPUTask *task, const Mat &frame, float *mean)
{
    Mat img_copy;
    int height = dpuGetInputTensorHeight(task, INPUT_NODE);
    int width = dpuGetInputTensorWidth(task, INPUT_NODE);
    int size = dpuGetInputTensorSize(task, INPUT_NODE);
    int8_t *data = dpuGetInputTensorAddress(task, INPUT_NODE);

    Mat img_res;
    img_res = Mat(height, width, CV_8SC3);
    //cout << "img.rows = "<<img.rows<<" img.cols = "<<img.cols<<endl;
    //cv::resize(img, img_res, Size(height, width),0,0,cv::INTER_LINEAR);
    if(frame.rows != height)
	{
        resize(frame, img_res, Size(height, width),0,0,cv::INTER_LINEAR);
    }
	else
	{
        img_res = frame;
    }

    /*
    for(int i_w = 0; i_w<3; i_w++)
	{
    	for(int i = 0; i<3; i++)
		{
    		cout << "ori img =" << (float)frame.at<Vec3b>(0,i_w)[i] << endl;
    		cout << "resize img = " << (float)img_res.at<Vec3b>(0,i_w)[i] << endl;
    	}
    }
    for(int i = 0; i<3; i++)
	{
    	cout << "1314 resize img = " << (float)img_res.at<Vec3b>(438, 0)[i] << endl;
    }*/


    float scale_fix = 0.5; //0.00390625f
    int value;
    float temp;
    int count = 0;
    //channel=1;
    if(img_res.channels() == 1)
	{
        for(int i_h = 0; i_h < img_res.rows; i_h++)
		{
            for(int i_w = 0; i_w < img_res.cols; i_w++)
			{
                temp = (img_res.at<int>(i_h,i_w) - mean[0])*scale_fix;
	            value = (int)my_round(temp);
                *(data++) = (int8_t)value;
	        }
        }
    }
    //channel=3
    else
    {
        for(int i_h = 0; i_h < img_res.rows; i_h++)
		{
            for(int i_w = 0; i_w < img_res.cols; i_w++)
			{
	            for(int i_c = 0; i_c < 3 ; i_c++)
				{
                    temp = ((float)img_res.at<Vec3b>(i_h,i_w)[i_c] - mean[i_c])*scale_fix;
	                temp = my_round(temp);
	                value = (int)temp;
	                if (count < 10)
					{
	                	//cout << "i =" << count << " img value = " << temp << endl;
	                }
	                count++;
	                //value = (int)my_round(temp);
	                if ((value > 127) || (value < -128))
					{ 
                       // cout << "error at i_h="<<i_h<<" i_w="<<i_w<<" i_c="<<i_c<<" value="<<value<<endl;
                    }
                    *(data++) = (int8_t)value;
	            }
            }
        }
    }
}
