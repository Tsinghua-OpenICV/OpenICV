#ifndef _SSD_DETECTION_H
#define _SSD_DETECTION_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Basis/icvStructureData.hxx"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"
#include "OpenICV/structure/icvCvMatData.hxx"
#include "OpenICV/Basis/icvSemaphore.h"

#include "OpenICV/structure/structureBBox2D.h"

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
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "setInputImage.h"
#include "postPre.h" 
#include <thread>                // std::thread
#include <mutex>                 // std::mutex, std::unique_lock
#include <condition_variable>    // std::condition_variable
#include "threadSafePriorQueue.h"
#include "prior.h"


using namespace icv;
using namespace core;
using namespace std;
using namespace cv;
using namespace chrono;

typedef data::icvStructureData<BoundingBoxes2d> icvbbox2d ;
typedef pair<int, Mat> imagePair;
class paircomp
{
public:
    bool operator()(const imagePair &n1, const imagePair &n2) const
    {
        if (n1.first == n2.first)
        {
            return (n1.first > n2.first);
        }

        return n1.first > n2.first;
    }
};

typedef struct showResult
{
    BoundingBoxes2d bboxes ;
    Mat image ;
}showResult;

typedef pair<int, showResult> showPair;
class paircomp_show
{
public:
    bool operator()(const showPair &n1, const showPair &n2) const
    {
        if (n1.first == n2.first)
        {
            return (n1.first > n2.first);
        }

        return n1.first > n2.first;
    }
};

class SSD : public icvFunction
{
public:
    SSD(icv_shared_ptr<const icvMetaData> info) : icvFunction(info) ,queueInput(30), queueShow(30), task(5)
    {
        timesync_=new icvSemaphore(1);
        icv_thread loop(bind(&SSD::InnerLoop, this));
        //load prior data
        prior_data = (float *)kPriorData;
        // prior_data = (float *)malloc(anchor_num * 8 * sizeof(float));
        // FILE *fp;
        // fp = fopen(priorbox_path, "rb");
        // ICV_LOG_INFO<<"open file success";
        // fseek(fp, 0, SEEK_END);
        // if (ftell(fp) != anchor_num * 8 * sizeof(float)){
        //     printf("file size = %d \n", ftell(fp));
        //     printf("error file size = %d \n", anchor_num * 8 * sizeof(float));
        // } 
        // fseek(fp, 0, SEEK_SET);
        // fread(prior_data, sizeof(float), anchor_num * 8, fp);
        // fclose(fp);
        // ICV_LOG_INFO<<"close file ";

        /* The main procress of using DPU kernel begin. */
        
        dpuOpen();
        ICV_LOG_INFO<<"dpu open";
        // Create the kernel 
        kernel = dpuLoadKernel("SSD_VGG_no_bn_0812");
        ICV_LOG_INFO<<"load kernel";

        /* Create 4 DPU Tasks for YOLO-v3 network model */
        generate(task.begin(), task.end(), std::bind(dpuCreateTask, kernel, 0));
        
        // thread(&SSD::readInput,this),
        threadsList[0] = make_shared<std::thread>(&SSD::displayFrame, this);
        threadsList[1] = make_shared<std::thread>(&SSD::runSSD, this, task[0], prior_data);
        threadsList[2] = make_shared<std::thread>(&SSD::runSSD, this, task[1], prior_data);
        threadsList[3] = make_shared<std::thread>(&SSD::runSSD, this, task[2], prior_data);
        // thread(runSSD, task[3], prior_data),
        // thread(runSSD, task[4], prior_data),
        box_send = new icvbbox2d();
        img_data = new  icv::data::icvCvMatData();
        start_ = icvTime::now_us().time_since_epoch().count();
        count_ = 0 ;
        
	}

    virtual ~SSD() {
        for (int i = 0; i < 4; i++) {
            threadsList[i]->join();
        }

        // Destroy prior data
        free(prior_data);

        // Destroy DPU Tasks & free resources
        for_each(task.begin(), task.end(), dpuDestroyTask);

        // Destroy the kernel after classification
        dpuDestroyKernel(kernel);

        dpuClose();
    }

    void displayFrame()
    {
        showPair tempShowPair ;
        while (true)
        {    
            if(queueShow.WaitDequeue2(&tempShowPair, idxShowImage))
            {
                box_send->setoutvalue(tempShowPair.second.bboxes);
                Send_Out(box_send,0);
                idxShowImage++;
                img_data->setoutvalue(tempShowPair.second.image);
		        Send_Out(img_data,1);
                count_ ++ ;
                end_ = icvTime::now_us().time_since_epoch().count();
                ICV_LOG_INFO<<"\n---Mean SSD process time: "<<(end_-start_)/1000/count_<<" ms";
                // imshow("SSD_result",tempShowPair.second.image);
                // waitKey(10);
            }
        }
    }


    void runSSD(DPUTask *task, float *prior_data)
    {
        /* mean values for SSD */
        float mean[3] = {104.0f, 117.0f, 123.0f};
        //input network image size
        int height = dpuGetInputTensorHeight(task, INPUT_NODE);
        int width = dpuGetInputTensorWidth(task, INPUT_NODE);
        BoundingBoxes2d boxes ;

        while (true)
        {
            // pair<int, Mat> pairIndexImage;
            imagePair pairIndexImage ;
            boxes.bounding_boxes.clear() ;

            /* get an input frame from input frames queue */
            queueInput.WaitDequeue(&pairIndexImage);
            
            //feed input frame into DPU Task with mean value,resize the origin image 
            dpuSetInputImageWithScale(task, INPUT_NODE, pairIndexImage.second, mean, 1.0f);
            /* invoke the running of DPU for SSD */
            dpuRunTask(task);

            int8_t *conf_data=dpuGetOutputTensorAddress(task, outputs_node[0].c_str());
            int8_t *loc_data=dpuGetOutputTensorAddress(task, outputs_node[1].c_str());
            float conf_scale = dpuGetOutputTensorScale(task, outputs_node[0].c_str());
            conf_scale = 1.0/conf_scale;
            float loc_scale = dpuGetOutputTensorScale(task, outputs_node[1].c_str());
            loc_scale = 1.0/loc_scale;

            SSDPostTreat(task, pairIndexImage.second, conf_data, conf_scale, loc_data, loc_scale, prior_data, boxes); 
            boxes.header.stamp = time(NULL);
            // boxes.header.frame_id = "ssd_detection";
        
            showPair show ;
            show.first = pairIndexImage.first ;
            show.second.bboxes = boxes ;
            show.second.image = pairIndexImage.second ;

            /* push the image into display frame queue */
            queueShow.Enqueue(show);
        }
    }


    
    virtual void Execute() override
    {
     
        // ICV_LOG_INFO<<"start input";
        Mat mFrame ;
        //timesync_->Lock();
        
        mFrame = read_Input<icv::data::icvCvMatData>(0);
        // ICV_LOG_INFO<<"finish input";
        if (is_not_empty(0))
        {
            cv::Mat img_resize ;
            cv::Rect rect(128,150,512,256);
            // ICV_LOG_INFO<<"received size: "<<mFrame.size();
            img_resize = mFrame(rect);
            // ICV_LOG_INFO<<"finish resize";
            queueInput.Enqueue(make_pair(idxInputImage++,img_resize));
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
    int idxInputImage = 0; // frame index of input video
    int idxShowImage = 0;  // next frame index to be displayed
    bool bReading = true;  // flag of reding input frame
    ThreadSafePriorQueue<imagePair, paircomp> queueInput;
    ThreadSafePriorQueue<showPair, paircomp_show> queueShow;
    cv::Mat mFrame ;
    icvbbox2d* box_send;
    std::array<std::shared_ptr<std::thread>, 5> threadsList;    
    float *prior_data;
    DPUKernel *kernel; 
    vector<DPUTask *> task;
    icv::data::icvCvMatData *img_data;
    time_t start_, end_ ;
    int count_ ;

};

ICV_REGISTER_FUNCTION(SSD)

#endif 
