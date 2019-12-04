#ifndef __PROFILE_H__
#define __PROFILE_H__
#include <assert.h>
#include <algorithm>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <atomic>
#include <sys/stat.h>
#include <unistd.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <queue>
#include <mutex>
#include <string>
#include <vector>
#include <thread>
#include <zconf.h>
#include <dnndk/dnndk.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
using namespace chrono;

#define top_k 200
#define KERNEL_CONV "SSD_VGG_no_bn_0812" 
#define INPUT_NODE "conv1_1"
const string outputs_node[2] = {"mbox_conf", "mbox_loc"};
#define num_class 21
const int anchor_num = 8732;//8542
const float confidence_threshold = 0.5f;
// 这三个数据是nms_threshold和图像的宽高
const float nms_threshold = 0.5f;
//origin image size
// const float im_or_width = 512;
// const float im_or_height = 256;
// const string classes[num_class-1] = {"car", "person", "cycle"};
const string classes[num_class] = {"background","aeroplane", "bicycle", "bird","boat",	"bottle","bus","car","cat","chair","cow","diningtable","dog",
	"horse","motorbike","person","pottedplant","sheep","sofa","train","tvmonitor"};

//priorbox path
const char priorbox_path[]  = "./prior.bin";

// 这是用来存储中间状态的结构体
struct IndexScoreBBox {
  bool flag;
  int index;
  float score;
  int category;
  float xmin;
  float ymin;
  float xmax;
  float ymax;
};
#endif
