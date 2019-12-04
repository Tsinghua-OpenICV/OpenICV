#ifndef __PAIRCOMP_H__
#define __PAIRCOMP_H__ 

#include <opencv2/imgproc/imgproc.hpp>		 
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>

#include "OpenICV/structure/structureLaneData.h"

// #include "lane_detection/ld_Frame.h"
using namespace cv;
using namespace std;

typedef struct TIMEINFO {
	unsigned long int time_s;
	unsigned long int time_ns;
} TIMEINFO;

typedef struct INPUTTASK {
	TIMEINFO timestamp;
	cv::Mat image;
} INPUTTASK;

typedef pair<int, INPUTTASK> InputTaskPair;
class paircomp_inputtask
{
public:
    bool operator()(const InputTaskPair &n1, const InputTaskPair &n2) const 
    {
        if (n1.first == n2.first) {
            return (n1.first > n2.first);
        }
        return n1.first > n2.first;
    }
};

typedef pair<int, Mat> imagePair;
class paircomp
{
public:
    bool operator()(const imagePair &n1, const imagePair &n2) const
    {
        if (n1.first == n2.first) {
            return (n1.first > n2.first);
        }
        return n1.first > n2.first;
    }
};

typedef pair<int, ld_Frame> FramePair;
class paircomp_ld
{
public:
    bool operator()(const FramePair &n1, const FramePair &n2) const
    {
        if (n1.first == n2.first) {
            return (n1.first > n2.first);
        }
        return n1.first > n2.first;
    }
};

// typedef pair<int, cv_bridge::CvImage> cvImagePair;
// class paircomp_cvimage
// {
// public:
//     bool operator()(const cvImagePair &n1, const cvImagePair &n2) const
//     {
//         if (n1.first == n2.first) {
//             return (n1.first > n2.first);
//         }
//         return n1.first > n2.first;
//     }
// };

#endif